#!/usr/bin/env python3
"""
Pick and Place Pucks
Grabs red, green and blue pucks one by one and drops each at its ArUco-marked corner.

Detection and approach mirror follow_red_puck.py: HSV mask on the colour, biggest contour,
turn proportional to image-x error, drive forward while the blob area is below a target
area, stop when it is bigger than that. Same idea is reused for the ArUco marker (using
its bounding-box area) and for the yellow drop square.

Mission: red -> green -> blue. ArUco IDs: 1=red, 2=green, 3=blue (DICT_4X4_50).
Gripper: publish UInt16 to /servo (open/close angles), read /servoLoad (Float32) for force.
"""

import math
import os
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image, Range
from std_msgs.msg import Float32, UInt16


class PickAndPlace:
    S_INIT = "INIT"
    S_START_SCAN = "START_SCAN"
    S_HUNT_PUCK = "HUNT_PUCK"
    S_GRAB = "GRAB"
    S_HUNT_ARUCO = "HUNT_ARUCO"
    S_CENTER_YELLOW = "CENTER_YELLOW"
    S_DROP = "DROP"
    S_DONE = "DONE"

    def __init__(self):
        rospy.init_node("pick_and_place", anonymous=True)

        self.bridge = CvBridge()
        self.has_display = "DISPLAY" in os.environ and os.environ["DISPLAY"]
        self.path_enabled = bool(rospy.get_param("~path_enabled", True))
        self.path_canvas_px = int(rospy.get_param("~path_canvas_px", 700))
        self.path_scale_px_per_m = float(rospy.get_param("~path_scale_px_per_m", 140.0))

        self.pickup_order = ["red", "green", "blue"]
        self.aruco_id_for = {"red": 1, "green": 2, "blue": 3}
        self.color_index = 0
        self.state = self.S_INIT
        self.state_t0 = rospy.Time.now()
        self.warmup_s = 2.0
        self.start_scan_s = 32.0

        self.rgb_frame = None
        self.servo_load = 0.0
        self.range_fl = float("inf")
        self.range_fr = float("inf")
        self.desired_servo = 0

        self.path_lock = threading.Lock()
        self.path_x = 0.0
        self.path_y = 0.0
        self.path_yaw = 0.0
        self.path_last_stamp = rospy.Time.now()
        self.path_last_lin = 0.0
        self.path_last_ang = 0.0
        self.path_points = [(0.0, 0.0)]

        self.min_blob_area = 300
        self.target_puck_area = 9000
        self.target_aruco_area = 12000
        self.target_yellow_area = 25000
        self.min_yellow_area = 500

        self.puck_grab_top_frac = 0.9

        self.max_lin = 0.15
        self.max_ang = 0.4
        self.k_turn = 0.7
        self.k_forward = 0.25
        self.search_ang = 0.2
        self.start_scan_ang = 0.2

        self.servo_open = 0
        self.servo_close = 155
        self.force_held_threshold = 1.0
        self.grip_settle_s = 1.5
        self.lunge_s = 1.0

        self.safe_front_distance_m = 0.20
        self.drop_distance_m = 0.30

        self.desired_servo = self.servo_open

        self.back_away_s = 5
        self.back_away_lin = -0.10

        self.hsv_ranges = {
            "red": [
                (np.array([0, 120, 70]), np.array([10, 255, 255])),
                (np.array([170, 120, 70]), np.array([179, 255, 255])),
            ],
            "green": [
                (np.array([35, 80, 60]), np.array([85, 255, 255])),
            ],
            "blue": [
                (np.array([90, 80, 60]), np.array([130, 255, 255])),
            ],
            "yellow": [
                (np.array([20, 100, 100]), np.array([35, 255, 255])),
            ],
        }
        self.kernel = np.ones((5, 5), np.uint8)

        self._aruco = self._make_aruco_detector(cv2.aruco.DICT_4X4_50)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=10)
        self.path_pub = rospy.Publisher("/pick_and_place/path_image", Image, queue_size=1, latch=True)

        rospy.Timer(rospy.Duration(0.1), self._republish_servo)
        rospy.Timer(rospy.Duration(0.2), self._path_timer)

        rospy.Subscriber("/camera/color/image_2fps/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber("/servoLoad", Float32, self.servo_load_callback)
        rospy.Subscriber("/range/fl", Range, self.range_fl_callback)
        rospy.Subscriber("/range/fr", Range, self.range_fr_callback)

        rospy.sleep(0.5)
        self.open_gripper()

        rospy.loginfo("Pick-and-place started. Order: %s", self.pickup_order)

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.rgb_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.tick()
        except Exception as e:
            rospy.logerr(f"Image error: {e}")

    def servo_load_callback(self, msg):
        self.servo_load = float(msg.data)

    def range_fl_callback(self, msg):
        self.range_fl = self._sanitize_range(msg.range)

    def range_fr_callback(self, msg):
        self.range_fr = self._sanitize_range(msg.range)

    def _sanitize_range(self, r):
        if r is None or r != r or r <= 0.0 or r > 10.0:
            return float("inf")
        return float(r)

    def front_distance(self):
        return min(self.range_fl, self.range_fr)

    def open_gripper(self):
        self.desired_servo = self.servo_open
        self.servo_pub.publish(UInt16(self.desired_servo))

    def close_gripper(self):
        self.desired_servo = self.servo_close
        self.servo_pub.publish(UInt16(self.desired_servo))

    def is_holding(self):
        return self.servo_load > self.force_held_threshold

    def _republish_servo(self, _evt):
        self.servo_pub.publish(UInt16(self.desired_servo))

    def stop(self):
        self._record_motion_command(0.0, 0.0)
        self.cmd_pub.publish(Twist())

    def drive(self, lin, ang):
        t = Twist()
        t.linear.x = max(-self.max_lin, min(self.max_lin, lin))
        t.angular.z = max(-self.max_ang, min(self.max_ang, ang))
        self._record_motion_command(t.linear.x, t.angular.z)
        self.cmd_pub.publish(t)
        return t.linear.x, t.angular.z

    def _record_motion_command(self, lin, ang):
        if not self.path_enabled:
            return
        with self.path_lock:
            self._integrate_path_locked(rospy.Time.now())
            self.path_last_lin = float(lin)
            self.path_last_ang = float(ang)

    def _path_timer(self, _evt):
        if not self.path_enabled:
            return
        with self.path_lock:
            self._integrate_path_locked(rospy.Time.now())
            image = self._render_path_locked()

        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(msg)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "path image publish failed: %s", exc)

        if self.has_display:
            cv2.imshow("Path Tracking", image)
            cv2.waitKey(1)

    def _integrate_path_locked(self, now):
        dt = (now - self.path_last_stamp).to_sec()
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)

        mid_yaw = self.path_yaw + 0.5 * self.path_last_ang * dt
        self.path_x += self.path_last_lin * math.cos(mid_yaw) * dt
        self.path_y += self.path_last_lin * math.sin(mid_yaw) * dt
        self.path_yaw += self.path_last_ang * dt
        self.path_yaw = math.atan2(math.sin(self.path_yaw), math.cos(self.path_yaw))
        self.path_last_stamp = now

        last_x, last_y = self.path_points[-1]
        if math.hypot(self.path_x - last_x, self.path_y - last_y) >= 0.01:
            self.path_points.append((self.path_x, self.path_y))
            if len(self.path_points) > 5000:
                self.path_points = self.path_points[-5000:]

    def _world_to_path_px(self, x, y):
        center = self.path_canvas_px // 2
        px = int(center + y * self.path_scale_px_per_m)
        py = int(center - x * self.path_scale_px_per_m)
        return px, py

    def _render_path_locked(self):
        size = self.path_canvas_px
        image = np.full((size, size, 3), 245, dtype=np.uint8)
        center = size // 2

        grid_step = max(20, int(0.25 * self.path_scale_px_per_m))
        for p in range(0, size, grid_step):
            cv2.line(image, (p, 0), (p, size), (225, 225, 225), 1)
            cv2.line(image, (0, p), (size, p), (225, 225, 225), 1)
        cv2.line(image, (center, 0), (center, size), (180, 180, 180), 1)
        cv2.line(image, (0, center), (size, center), (180, 180, 180), 1)

        if len(self.path_points) >= 2:
            pts = np.array([self._world_to_path_px(x, y) for x, y in self.path_points], dtype=np.int32)
            cv2.polylines(image, [pts], False, (255, 80, 0), 2)

        origin = self._world_to_path_px(0.0, 0.0)
        robot = self._world_to_path_px(self.path_x, self.path_y)
        heading = self._world_to_path_px(
            self.path_x + 0.20 * math.cos(self.path_yaw),
            self.path_y + 0.20 * math.sin(self.path_yaw),
        )

        cv2.circle(image, origin, 5, (0, 160, 0), -1)
        cv2.circle(image, robot, 7, (0, 0, 255), -1)
        cv2.arrowedLine(image, robot, heading, (0, 0, 180), 2, tipLength=0.35)

        color = self.pickup_order[self.color_index] if self.color_index < len(self.pickup_order) else "done"
        cv2.putText(image, "approx path from /cmd_vel", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 2)
        cv2.putText(image, f"state={self.state} target={color}", (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1)
        cv2.putText(image, f"x={self.path_x:+.2f}m y={self.path_y:+.2f}m yaw={math.degrees(self.path_yaw):+.0f}deg", (12, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1)
        cv2.putText(image, "green=start red=robot blue=trail", (12, size - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1)
        return image

    def color_mask(self, hsv, color):
        m = None
        for low, high in self.hsv_ranges[color]:
            part = cv2.inRange(hsv, low, high)
            m = part if m is None else (m + part)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel)
        return m

    def biggest_blob(self, mask, min_area):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        if area < min_area:
            return None
        x, y, w, h = cv2.boundingRect(biggest)
        return {"cx": x + w // 2, "cy": y + h // 2, "x": x, "y": y, "w": w, "h": h, "area": area}

    def _make_aruco_detector(self, dict_id):
        if hasattr(cv2.aruco, "ArucoDetector"):
            d = cv2.aruco.getPredefinedDictionary(dict_id)
            p = cv2.aruco.DetectorParameters()
            return ("new", cv2.aruco.ArucoDetector(d, p))
        d = cv2.aruco.Dictionary_get(dict_id)
        p = cv2.aruco.DetectorParameters_create()
        return ("old", (d, p))

    def detect_aruco(self, frame, target_id):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        api, obj = self._aruco
        if api == "new":
            corners, ids, _ = obj.detectMarkers(gray)
        else:
            d, p = obj
            corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
        if ids is None:
            return None
        ids = ids.flatten().tolist()
        for i, mid in enumerate(ids):
            if mid == target_id:
                pts = corners[i].reshape(-1, 2)
                cx = int(pts[:, 0].mean())
                cy = int(pts[:, 1].mean())
                area = float(cv2.contourArea(pts.astype(np.float32)))
                return {"cx": cx, "cy": cy, "area": area, "pts": pts}
        return None

    def _approach(self, target, target_area, img_w):
        err_x = (target["cx"] - img_w / 2.0) / (img_w / 2.0)
        ang = -self.k_turn * err_x
        front = self.front_distance()

        if target["area"] >= target_area:
            self.stop()
            return (f"stop (area={int(target['area'])} >= {target_area})", True)

        if front < self.safe_front_distance_m and target["area"] >= 0.5 * target_area:
            self.stop()
            return (f"stop (IR {front:.2f}m + area {int(target['area'])} confirm)", True)

        lin = self.k_forward * (1.0 - target["area"] / target_area)
        lin_c, ang_c = self.drive(lin, ang)
        return (self._motion_label(lin_c, ang_c, err_x, target["area"]) + f" front={front:.2f}m", False)

    def _motion_label(self, lin, ang, err=None, area=None):
        if abs(lin) < 0.01 and abs(ang) < 0.01:
            base = "stop"
        elif abs(ang) < 0.15 and lin > 0:
            base = "forward"
        elif abs(ang) < 0.15 and lin < 0:
            base = "backward"
        elif abs(lin) < 0.05 and ang > 0:
            base = "spin left"
        elif abs(lin) < 0.05 and ang < 0:
            base = "spin right"
        elif lin > 0:
            base = "forward + turn " + ("left" if ang > 0 else "right")
        else:
            base = "backward + turn " + ("left" if ang > 0 else "right")
        extras = []
        if err is not None:
            extras.append(f"err={err:+.2f}")
        if area is not None:
            extras.append(f"area={int(area)}")
        return base + (" (" + ", ".join(extras) + ")" if extras else "")

    def tick(self):
        if self.rgb_frame is None:
            return

        frame = self.rgb_frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_h, img_w = frame.shape[:2]

        if self.color_index >= len(self.pickup_order):
            self.state = self.S_DONE

        if self.state == self.S_DONE:
            self.stop()
            rospy.loginfo_throttle(2.0, "DONE — all pucks delivered")
            self._show(frame, "DONE", None)
            return

        target_color = self.pickup_order[self.color_index]
        target_aruco = self.aruco_id_for[target_color]
        action = ""
        overlay_blob = None
        overlay_aruco = None

        if self.state == self.S_INIT:
            self.stop()
            self.open_gripper()
            elapsed = (rospy.Time.now() - self.state_t0).to_sec()
            action = f"warming up... t={elapsed:.1f}s / {self.warmup_s}s"
            if elapsed > self.warmup_s:
                rospy.loginfo("Warmup done, doing startup 360 scan")
                self._goto(self.S_START_SCAN)

        elif self.state == self.S_START_SCAN:
            elapsed = (rospy.Time.now() - self.state_t0).to_sec()
            if elapsed < self.start_scan_s:
                lin_c, ang_c = self.drive(0.0, self.start_scan_ang)
                action = f"startup 360 scan t={elapsed:.1f}s/{self.start_scan_s:.1f}s — {self._motion_label(lin_c, ang_c)}"
            else:
                self.stop()
                rospy.loginfo("Startup 360 scan done, beginning puck search")
                self._goto(self.S_HUNT_PUCK)
                action = "startup scan complete"

        elif self.state == self.S_HUNT_PUCK:
            blob = self.biggest_blob(self.color_mask(hsv, target_color), self.min_blob_area)
            overlay_blob = blob
            if blob is None:
                front = self.front_distance()
                lin_c, ang_c = self.drive(0.0, self.search_ang)
                action = f"searching {target_color} — {self._motion_label(lin_c, ang_c)} front={front:.2f}m"
            else:
                _thr = self.puck_grab_top_frac * img_h
                rospy.loginfo(
                    f"[{target_color}] bbox top={blob['y']} bottom={blob['y'] + blob['h']} "
                    f"height={blob['h']} img_h={img_h} threshold={_thr:.0f} "
                    f"=> compare top({blob['y']}) > thr({_thr:.0f}) ? {blob['y'] > _thr}"
                )
                if blob["y"] > self.puck_grab_top_frac * img_h:
                    self.stop()
                    rospy.loginfo(
                        f"[{target_color}] puck at gripper level "
                        f"(top={blob['y']}/{img_h}, area={int(blob['area'])}), grabbing"
                    )
                    self._goto(self.S_GRAB)
                    action = "grab on position"
                else:
                    label, reached = self._approach(blob, self.target_puck_area, img_w)
                    action = label
                    if reached:
                        rospy.loginfo(f"[{target_color}] in range, grabbing (publishing servo={self.servo_close})")
                        self._goto(self.S_GRAB)

        elif self.state == self.S_GRAB:
            elapsed = (rospy.Time.now() - self.state_t0).to_sec()
            if elapsed < self.lunge_s:
                lin_c, ang_c = self.drive(self.k_forward, 0.0)
                action = (
                    f"lunging into puck (t={elapsed:.1f}s/{self.lunge_s}s) — "
                    f"{self._motion_label(lin_c, ang_c)}"
                )
            else:
                self.stop()
                self.close_gripper()
                grip_elapsed = elapsed - self.lunge_s
                action = f"closing gripper (load={self.servo_load:.2f}, t={grip_elapsed:.1f}s)"
                if grip_elapsed > self.grip_settle_s:
                    if self.is_holding():
                        rospy.loginfo(f"[{target_color}] held (load={self.servo_load:.2f})")
                    else:
                        rospy.logwarn(
                            f"[{target_color}] grab unconfirmed (load={self.servo_load:.2f}, "
                            f"threshold {self.force_held_threshold}); proceeding anyway"
                        )
                    self._goto(self.S_HUNT_ARUCO)

        elif self.state == self.S_HUNT_ARUCO:
            mk = self.detect_aruco(frame, target_aruco)
            overlay_aruco = mk
            if mk is None:
                lin_c, ang_c = self.drive(0.0, self.search_ang)
                action = f"searching ArUco {target_aruco} — " + self._motion_label(lin_c, ang_c)
            else:
                label, reached = self._approach(mk, self.target_aruco_area, img_w)
                action = f"toward ArUco {target_aruco}: " + label
                if reached:
                    rospy.loginfo(f"[{target_color}] near ArUco, aligning on yellow")
                    self._goto(self.S_CENTER_YELLOW)

        elif self.state == self.S_CENTER_YELLOW:
            front = self.front_distance()
            yellow = self.biggest_blob(self.color_mask(hsv, "yellow"), self.min_yellow_area)
            mk = self.detect_aruco(frame, target_aruco)
            overlay_blob = yellow
            overlay_aruco = mk

            if front < self.drop_distance_m:
                self.stop()
                rospy.loginfo(
                    f"[{target_color}] within drop distance "
                    f"(front={front:.2f}m < {self.drop_distance_m}m), dropping"
                )
                self._goto(self.S_DROP)
            else:
                cue = yellow if yellow else mk
                cue_name = "yellow" if yellow else ("ArUco" if mk else None)
                if cue is None:
                    lin_c, ang_c = self.drive(0.0, self.search_ang)
                    action = f"searching corner cue — {self._motion_label(lin_c, ang_c)} front={front:.2f}m"
                else:
                    err_x = (cue["cx"] - img_w / 2.0) / (img_w / 2.0)
                    ang = -self.k_turn * err_x
                    lin_c, ang_c = self.drive(self.k_forward * 0.6, ang)
                    action = (
                        f"approach corner via {cue_name}: "
                        f"{self._motion_label(lin_c, ang_c, err_x, cue['area'])} "
                        f"front={front:.2f}m"
                    )

        elif self.state == self.S_DROP:
            self.open_gripper()
            elapsed = (rospy.Time.now() - self.state_t0).to_sec()
            if elapsed < self.grip_settle_s:
                self.stop()
                action = f"opening gripper (t={elapsed:.1f}s)"
            elif elapsed < self.grip_settle_s + self.back_away_s:
                lin_c, ang_c = self.drive(self.back_away_lin, 0.0)
                action = "backing away — " + self._motion_label(lin_c, ang_c)
            else:
                rospy.loginfo(f"[{target_color}] delivered")
                self.color_index += 1
                if self.color_index >= len(self.pickup_order):
                    self._goto(self.S_DONE)
                else:
                    rospy.loginfo("Next target: %s", self.pickup_order[self.color_index])
                    self._goto(self.S_HUNT_PUCK)
                action = "transitioning"

        rospy.loginfo(f"[{target_color}] {self.state}: {action}")

        self._show(frame, f"{self.state} -> {target_color}", action, blob=overlay_blob, aruco=overlay_aruco)

    def _goto(self, new_state):
        self.state = new_state
        self.state_t0 = rospy.Time.now()

    def _show(self, frame, state_label, action_label, blob=None, aruco=None):
        if not self.has_display:
            return
        if blob is not None:
            cv2.rectangle(frame, (blob["x"], blob["y"]), (blob["x"] + blob["w"], blob["y"] + blob["h"]), (0, 255, 255), 2)
            cv2.circle(frame, (blob["cx"], blob["cy"]), 5, (0, 255, 255), -1)
        if aruco is not None:
            pts = aruco["pts"].astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (aruco["cx"], aruco["cy"]), 5, (0, 255, 0), -1)
        cv2.putText(frame, state_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if action_label:
            cv2.putText(frame, action_label, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(frame, f"load={self.servo_load:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.imshow("Pick&Place", frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        self.stop()
        self.open_gripper()
        if self.has_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        PickAndPlace().run()
    except rospy.ROSInterruptException:
        pass
