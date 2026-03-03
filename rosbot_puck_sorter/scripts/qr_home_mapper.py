#!/usr/bin/env python3
import math
import threading

import actionlib
import cv2
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from rosbot_puck_sorter.msg import HomeBase, HomeBaseArray
from rosbot_puck_sorter.srv import ScanHomes, ScanHomesResponse


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class QRHomeMapper:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.corner_waypoints = rospy.get_param("~corner_waypoints", [])
        self.marker_mode = str(rospy.get_param("~marker_mode", "aruco")).strip().lower()
        self.qr_expected_codes = set(rospy.get_param("~qr_expected_codes", ["HOME_RED", "HOME_GREEN", "HOME_BLUE"]))
        self.aruco_dictionary_name = rospy.get_param("~aruco_dictionary", "DICT_4X4_50")
        self.aruco_id_to_color = self._load_aruco_color_map(rospy.get_param("~aruco_id_to_color", {"0": "red", "1": "green", "2": "blue"}))
        self.scan_timeout_s = rospy.get_param("~scan_timeout_s", 12.0)
        self.scan_retries = int(rospy.get_param("~scan_retries", 2))
        self.stable_reads_required = int(rospy.get_param("~stable_reads_required", 3))
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.start_frame = rospy.get_param("~start_frame", "start")
        self.publish_start_relative = bool(rospy.get_param("~publish_start_relative", True))

        self.latest_bgr = None
        self.latest_stamp = rospy.Time(0)
        self.detector = cv2.QRCodeDetector()
        self.aruco_dict = None
        self.aruco_detector = None
        self.aruco_params = None
        self._init_aruco_detector()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_homes = rospy.Publisher("/home_bases", HomeBaseArray, queue_size=1, latch=True)
        self.pub_homes_start = rospy.Publisher("/home_bases_start", HomeBaseArray, queue_size=1, latch=True)
        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for /move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(10.0))

        self.home_map = {}
        self.service = rospy.Service("/scan_homes", ScanHomes, self._scan_cb)

        rospy.loginfo("qr_home_mapper ready")

    def _load_aruco_color_map(self, raw_map):
        out = {}
        for k, v in raw_map.items():
            try:
                mk = int(k)
            except Exception:
                continue
            color = str(v).strip().lower()
            if color in ("red", "green", "blue"):
                out[mk] = color
        return out

    def _init_aruco_detector(self):
        if self.marker_mode != "aruco":
            return
        if not hasattr(cv2, "aruco"):
            rospy.logerr("marker_mode=aruco but OpenCV aruco module is unavailable")
            return

        dict_id = getattr(cv2.aruco, self.aruco_dictionary_name, None)
        if dict_id is None:
            rospy.logwarn("unknown aruco dictionary %s, falling back to DICT_4X4_50", self.aruco_dictionary_name)
            dict_id = cv2.aruco.DICT_4X4_50

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_detector = None

    def _image_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self.lock:
            self.latest_bgr = bgr
            self.latest_stamp = msg.header.stamp

    @staticmethod
    def _payload_to_color(payload):
        if payload.endswith("RED"):
            return "red"
        if payload.endswith("GREEN"):
            return "green"
        if payload.endswith("BLUE"):
            return "blue"
        return ""

    def _move_to(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = yaw_to_quat(yaw)

        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(45.0))
        if not finished:
            return False
        return self.move_base.get_state() == GoalStatus.SUCCEEDED

    def _read_qr_markers(self):
        with self.lock:
            if self.latest_bgr is None:
                return []
            frame = self.latest_bgr.copy()

        payloads = []

        ok, decoded, _points, _ = self.detector.detectAndDecodeMulti(frame)
        if ok and decoded is not None:
            for d in decoded:
                if d:
                    payloads.append(d.strip())

        if not payloads:
            single, _pts, _ = self.detector.detectAndDecode(frame)
            if single:
                payloads.append(single.strip())

        out = []
        for payload in payloads:
            color = self._payload_to_color(payload)
            if not color:
                continue
            out.append({"id": payload, "payload": payload, "color": color})
        return out

    def _read_aruco_markers(self):
        with self.lock:
            if self.latest_bgr is None:
                return []
            frame = self.latest_bgr.copy()

        if self.aruco_dict is None:
            return []

        if self.aruco_detector is not None:
            _corners, ids, _rejected = self.aruco_detector.detectMarkers(frame)
        else:
            _corners, ids, _rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        out = []
        if ids is None:
            return out

        for marker_id in ids.flatten().tolist():
            color = self.aruco_id_to_color.get(int(marker_id))
            if not color:
                continue
            out.append({"id": f"ARUCO_{int(marker_id)}", "payload": f"ARUCO_{int(marker_id)}", "color": color})
        return out

    def _read_markers(self):
        if self.marker_mode == "aruco":
            return self._read_aruco_markers()
        return self._read_qr_markers()

    def _scan_corner(self, remaining_colors):
        stable_id = ""
        stable_count = 0
        end_t = rospy.Time.now() + rospy.Duration(self.scan_timeout_s)

        while rospy.Time.now() < end_t and not rospy.is_shutdown():
            detections = self._read_markers()
            matches = [d for d in detections if d["color"] in remaining_colors]
            if matches:
                det = matches[0]
                det_id = det["id"]
                if det_id == stable_id:
                    stable_count += 1
                else:
                    stable_id = det_id
                    stable_count = 1

                if stable_count >= self.stable_reads_required:
                    return det
            else:
                stable_id = ""
                stable_count = 0

            rospy.sleep(0.10)

        return None

    def _publish_homes(self):
        msg = HomeBaseArray()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg_start = HomeBaseArray()
        msg_start.header = Header(stamp=rospy.Time.now(), frame_id=self.start_frame)
        for color, entry in self.home_map.items():
            h = HomeBase()
            h.header = msg.header
            h.color = color
            h.pose_map = entry["pose"]
            h.qr_payload = entry["payload"]
            msg.homes.append(h)

            if self.publish_start_relative:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = msg_start.header.stamp
                pose_msg.header.frame_id = "map"
                pose_msg.pose = entry["pose"]
                try:
                    out = self.tf_buffer.transform(pose_msg, self.start_frame, timeout=rospy.Duration(0.1))
                    hs = HomeBase()
                    hs.header = msg_start.header
                    hs.color = color
                    hs.pose_map = out.pose
                    hs.qr_payload = entry["payload"]
                    msg_start.homes.append(hs)
                except Exception:
                    rospy.logwarn_throttle(2.0, "start frame unavailable: could not transform home base to %s", self.start_frame)
        self.pub_homes.publish(msg)
        if self.publish_start_relative:
            self.pub_homes_start.publish(msg_start)

    def _scan_cb(self, req):
        if not req.start:
            return ScanHomesResponse(success=False, message="start flag false")

        if len(self.corner_waypoints) < 3:
            return ScanHomesResponse(success=False, message="corner_waypoints must contain 3 entries")

        if self.marker_mode == "aruco":
            expected_colors = set(self.aruco_id_to_color.values())
            if not expected_colors:
                return ScanHomesResponse(success=False, message="aruco_id_to_color is empty or invalid")
        else:
            expected_colors = set([self._payload_to_color(code) for code in self.qr_expected_codes if self._payload_to_color(code)])
            if not expected_colors:
                return ScanHomesResponse(success=False, message="qr_expected_codes has no parseable colors")

        remaining_colors = set(expected_colors)
        self.home_map = {}

        for i, wp in enumerate(self.corner_waypoints[:3]):
            x = float(wp.get("x", 0.0))
            y = float(wp.get("y", 0.0))
            yaw = float(wp.get("yaw", 0.0))

            moved = self._move_to(x, y, yaw)
            if not moved:
                return ScanHomesResponse(success=False, message=f"failed to navigate to corner {i}")

            detection = None
            for _ in range(self.scan_retries + 1):
                detection = self._scan_corner(remaining_colors)
                if detection is not None:
                    break

            if detection is None:
                return ScanHomesResponse(success=False, message=f"failed to scan marker at corner {i}")

            color = detection["color"]
            if color in self.home_map:
                return ScanHomesResponse(success=False, message=f"duplicate color {color} detected")

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation = yaw_to_quat(yaw)
            self.home_map[color] = {"pose": pose, "payload": detection["payload"]}
            remaining_colors.discard(color)

        if len(self.home_map) != 3:
            return ScanHomesResponse(success=False, message="did not map all 3 homes")

        self._publish_homes()
        return ScanHomesResponse(success=True, message="homes scanned and published")


def main():
    rospy.init_node("qr_home_mapper")
    QRHomeMapper()
    rospy.spin()


if __name__ == "__main__":
    main()
