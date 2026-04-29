#!/usr/bin/env python3
import math
import threading

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import UInt16


COLOR_RANGES = {
    "red": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([40, 60, 60]), np.array([85, 255, 255])),
    ],
    "blue": [
        (np.array([100, 80, 60]), np.array([130, 255, 255])),
    ],
}

DRAW_COLORS = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
}

TASKS = [
    ("red", 1),
    ("green", 2),
    ("blue", 3),
]

SEARCH_PUCK = "SEARCH_PUCK"
APPROACH_PUCK = "APPROACH_PUCK"
COMMIT_PICK = "COMMIT_PICK"
GRASP = "GRASP"
BACKUP_PICK = "BACKUP_PICK"
SEARCH_MARKER = "SEARCH_MARKER"
APPROACH_MARKER = "APPROACH_MARKER"
RELEASE = "RELEASE"
BACKUP_PLACE = "BACKUP_PLACE"
DONE = "DONE"


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class SimpleChallenge:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_2fps/compressed")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.servo_topic = rospy.get_param("~servo_topic", "/servo")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")

        self.open_angle = int(rospy.get_param("~open_angle", 0))
        self.close_angle = int(rospy.get_param("~close_angle", 170))

        self.min_area = float(rospy.get_param("~min_area", 300.0))
        self.aim_offset_px = float(rospy.get_param("~aim_offset_px", 50.0))
        self.puck_search_speed = float(rospy.get_param("~puck_search_speed", 0.30))
        self.marker_search_speed = float(rospy.get_param("~marker_search_speed", 0.40))
        self.max_linear = float(rospy.get_param("~max_linear", 0.15))
        self.max_angular = float(rospy.get_param("~max_angular", 0.80))
        self.kp_angular = float(rospy.get_param("~kp_angular", 0.005))
        self.kp_linear = float(rospy.get_param("~kp_linear", 0.003))
        self.approach_base = float(rospy.get_param("~approach_base", 0.06))

        self.commit_radius_px = float(rospy.get_param("~commit_radius_px", 70.0))
        self.commit_lost_min_radius_px = float(rospy.get_param("~commit_lost_min_radius_px", 38.0))
        self.commit_speed = float(rospy.get_param("~commit_speed", 0.08))
        self.commit_time = float(rospy.get_param("~commit_time", 1.8))

        self.grasp_settle = float(rospy.get_param("~grasp_settle", 1.5))
        self.backup_time = float(rospy.get_param("~backup_time", 1.0))
        self.backup_speed = float(rospy.get_param("~backup_speed", -0.06))

        self.tag_target_size_px = float(rospy.get_param("~tag_target_size_px", 180.0))
        self.tag_kp_angular = float(rospy.get_param("~tag_kp_angular", 0.005))
        self.tag_kp_linear = float(rospy.get_param("~tag_kp_linear", 0.002))
        self.tag_approach_base = float(rospy.get_param("~tag_approach_base", 0.06))

        self.front_stop_distance = float(rospy.get_param("~front_stop_distance", 0.08))
        self.front_slow_distance = float(rospy.get_param("~front_slow_distance", 0.18))
        self.scan_front_angle = math.radians(float(rospy.get_param("~scan_front_angle_deg", 35.0)))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.75))

        self.show_debug = bool(rospy.get_param("~show_debug", False))

        self.lock = threading.Lock()
        self.frame = None
        self.front_distance = None
        self.front_stamp = rospy.Time(0)

        self.task_index = 0
        self.state = SEARCH_PUCK
        self.state_start = rospy.Time.now()
        self.last_puck_radius = 0.0

        self.aruco_detect = self._make_aruco_detector()

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.servo_pub = rospy.Publisher(self.servo_topic, UInt16, queue_size=1, latch=True)
        rospy.Subscriber(self.image_topic, CompressedImage, self._image_cb, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

        rospy.on_shutdown(self.stop)
        rospy.sleep(0.5)
        self.set_gripper(self.open_angle)
        rospy.loginfo("simple_challenge ready: red->1, green->2, blue->3")

    def _make_aruco_detector(self):
        if not hasattr(cv2, "aruco"):
            rospy.logerr("OpenCV aruco module is missing")
            return None

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            return lambda gray: detector.detectMarkers(gray)

        params = cv2.aruco.DetectorParameters_create()
        return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    def _image_cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self.lock:
            self.frame = frame

    def _scan_cb(self, msg):
        values = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if abs(angle) <= self.scan_front_angle and np.isfinite(distance):
                if distance >= msg.range_min and (msg.range_max <= 0.0 or distance <= msg.range_max):
                    values.append(float(distance))
            angle += msg.angle_increment
        with self.lock:
            self.front_distance = min(values) if values else None
            self.front_stamp = rospy.Time.now()

    def latest_frame(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def set_state(self, state):
        if state != self.state:
            if self.task_index < len(TASKS):
                color, marker_id = self.current_task()
            else:
                color, marker_id = "complete", 0
            rospy.loginfo("%s marker %d: %s -> %s", color, marker_id, self.state, state)
        self.state = state
        self.state_start = rospy.Time.now()

    def current_task(self):
        return TASKS[self.task_index]

    def elapsed(self):
        return (rospy.Time.now() - self.state_start).to_sec()

    def set_gripper(self, angle):
        self.servo_pub.publish(UInt16(data=int(clamp(angle, 0, 180))))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        linear_x = clamp(linear_x, -self.max_linear, self.max_linear)
        angular_z = clamp(angular_z, -self.max_angular, self.max_angular)

        if linear_x > 0.0:
            linear_x *= self.forward_scale()

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def forward_scale(self):
        with self.lock:
            distance = self.front_distance
            stamp = self.front_stamp
        if distance is None or (rospy.Time.now() - stamp).to_sec() > self.scan_timeout:
            return 1.0
        if distance <= self.front_stop_distance:
            rospy.logwarn_throttle(1.0, "front obstacle %.2fm: stopping forward motion", distance)
            return 0.0
        if distance >= self.front_slow_distance:
            return 1.0
        span = max(0.001, self.front_slow_distance - self.front_stop_distance)
        return clamp((distance - self.front_stop_distance) / span, 0.0, 1.0)

    def find_largest_puck(self, frame, color):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in COLOR_RANGES[color]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            if area > best_area:
                best = contour
                best_area = area
        return best

    def find_marker(self, frame, marker_id):
        if self.aruco_detect is None:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detect(gray)
        if ids is None:
            return None
        for i, seen_id in enumerate(ids.flatten()):
            if int(seen_id) != marker_id:
                continue
            points = corners[i].reshape(-1, 2)
            cx = int(points[:, 0].mean())
            cy = int(points[:, 1].mean())
            sides = [np.linalg.norm(points[j] - points[(j + 1) % 4]) for j in range(4)]
            return cx, cy, int(max(sides)), points.astype(np.int32)
        return None

    def step(self, frame):
        color, marker_id = self.current_task()
        height, width = frame.shape[:2]
        aim_x = int(width / 2 + self.aim_offset_px)

        if self.state in (SEARCH_PUCK, APPROACH_PUCK, COMMIT_PICK, GRASP, BACKUP_PICK):
            self.step_pick(frame, color, aim_x)
        else:
            self.step_place(frame, color, marker_id, aim_x)

        if self.show_debug:
            self.draw_debug(frame, color, marker_id, aim_x)

    def step_pick(self, frame, color, aim_x):
        contour = self.find_largest_puck(frame, color)
        cx = 0
        radius = 0
        if contour is not None:
            (fx, _fy), fr = cv2.minEnclosingCircle(contour)
            cx = int(fx)
            radius = int(fr)
            self.last_puck_radius = radius

        if self.state == SEARCH_PUCK:
            if contour is not None:
                self.set_state(APPROACH_PUCK)
            else:
                self.publish_cmd(0.0, self.puck_search_speed)
            return

        if self.state == APPROACH_PUCK:
            if contour is None:
                if self.last_puck_radius >= self.commit_lost_min_radius_px:
                    self.set_state(COMMIT_PICK)
                else:
                    self.last_puck_radius = 0.0
                    self.set_state(SEARCH_PUCK)
                return
            if radius >= self.commit_radius_px:
                self.set_state(COMMIT_PICK)
                return
            heading_error = cx - aim_x
            angular = -self.kp_angular * heading_error
            linear = self.approach_base + self.kp_linear * (self.commit_radius_px - radius)
            self.publish_cmd(linear, angular)
            return

        if self.state == COMMIT_PICK:
            if self.elapsed() < self.commit_time:
                self.publish_cmd(self.commit_speed, 0.0)
            else:
                self.stop()
                self.set_gripper(self.close_angle)
                self.set_state(GRASP)
            return

        if self.state == GRASP:
            self.set_gripper(self.close_angle)
            self.stop()
            if self.elapsed() >= self.grasp_settle:
                self.set_state(BACKUP_PICK)
            return

        if self.state == BACKUP_PICK:
            if self.elapsed() < self.backup_time:
                self.publish_cmd(self.backup_speed, 0.0)
            else:
                self.stop()
                self.set_state(SEARCH_MARKER)

    def step_place(self, frame, color, marker_id, aim_x):
        tag = self.find_marker(frame, marker_id)

        if self.state == SEARCH_MARKER:
            if tag is not None:
                self.set_state(APPROACH_MARKER)
            else:
                self.publish_cmd(0.0, self.marker_search_speed)
            return

        if self.state == APPROACH_MARKER:
            if tag is None:
                self.set_state(SEARCH_MARKER)
                return
            cx, _cy, size, _points = tag
            if size >= self.tag_target_size_px:
                self.stop()
                self.set_state(RELEASE)
                return
            heading_error = cx - aim_x
            angular = -self.tag_kp_angular * heading_error
            linear = self.tag_approach_base + self.tag_kp_linear * (self.tag_target_size_px - size)
            self.publish_cmd(linear, angular)
            return

        if self.state == RELEASE:
            self.set_gripper(self.open_angle)
            self.stop()
            if self.elapsed() >= self.grasp_settle:
                self.set_state(BACKUP_PLACE)
            return

        if self.state == BACKUP_PLACE:
            if self.elapsed() < self.backup_time:
                self.publish_cmd(self.backup_speed, 0.0)
                return
            self.stop()
            rospy.loginfo("finished %s puck", color)
            self.task_index += 1
            if self.task_index >= len(TASKS):
                self.set_state(DONE)
                return
            self.last_puck_radius = 0.0
            self.set_state(SEARCH_PUCK)

    def draw_debug(self, frame, color, marker_id, aim_x):
        contour = self.find_largest_puck(frame, color)
        if contour is not None:
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            cv2.circle(frame, (int(cx), int(cy)), int(radius), DRAW_COLORS[color], 2)
        tag = self.find_marker(frame, marker_id)
        if tag is not None:
            cx, cy, _size, points = tag
            cv2.polylines(frame, [points], True, (0, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)
        cv2.line(frame, (aim_x, 0), (aim_x, frame.shape[0]), (255, 255, 0), 1)
        cv2.putText(frame, f"{color}->{marker_id} {self.state}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("simple_challenge", frame)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == DONE:
                self.stop()
                rospy.loginfo("challenge complete")
                break
            frame = self.latest_frame()
            if frame is None:
                rospy.logwarn_throttle(2.0, "waiting for camera image on %s", self.image_topic)
                self.stop()
            else:
                self.step(frame)
            rate.sleep()
        self.stop()
        if self.show_debug:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("simple_challenge")
    SimpleChallenge().run()


if __name__ == "__main__":
    main()
