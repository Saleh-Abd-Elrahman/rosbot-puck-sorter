#!/usr/bin/env python3
import math
import threading

import actionlib
import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Quaternion
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
        self.qr_expected_codes = set(rospy.get_param("~qr_expected_codes", ["HOME_RED", "HOME_GREEN", "HOME_BLUE"]))
        self.scan_timeout_s = rospy.get_param("~scan_timeout_s", 12.0)
        self.scan_retries = int(rospy.get_param("~scan_retries", 2))
        self.stable_reads_required = int(rospy.get_param("~stable_reads_required", 3))
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")

        self.latest_bgr = None
        self.latest_stamp = rospy.Time(0)
        self.detector = cv2.QRCodeDetector()

        self.pub_homes = rospy.Publisher("/home_bases", HomeBaseArray, queue_size=1, latch=True)
        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for /move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(10.0))

        self.home_map = {}
        self.service = rospy.Service("/scan_homes", ScanHomes, self._scan_cb)

        rospy.loginfo("qr_home_mapper ready")

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
        return finished

    def _read_qr(self):
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

        return payloads

    def _scan_corner(self, remaining_payloads):
        stable_payload = ""
        stable_count = 0
        end_t = rospy.Time.now() + rospy.Duration(self.scan_timeout_s)

        while rospy.Time.now() < end_t and not rospy.is_shutdown():
            payloads = self._read_qr()
            matches = [p for p in payloads if p in remaining_payloads]
            if matches:
                payload = matches[0]
                if payload == stable_payload:
                    stable_count += 1
                else:
                    stable_payload = payload
                    stable_count = 1

                if stable_count >= self.stable_reads_required:
                    return payload
            else:
                stable_payload = ""
                stable_count = 0

            rospy.sleep(0.10)

        return ""

    def _publish_homes(self):
        msg = HomeBaseArray()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        for color, entry in self.home_map.items():
            h = HomeBase()
            h.header = msg.header
            h.color = color
            h.pose_map = entry["pose"]
            h.qr_payload = entry["payload"]
            msg.homes.append(h)
        self.pub_homes.publish(msg)

    def _scan_cb(self, req):
        if not req.start:
            return ScanHomesResponse(success=False, message="start flag false")

        if len(self.corner_waypoints) < 3:
            return ScanHomesResponse(success=False, message="corner_waypoints must contain 3 entries")

        remaining_payloads = set(self.qr_expected_codes)
        self.home_map = {}

        for i, wp in enumerate(self.corner_waypoints[:3]):
            x = float(wp.get("x", 0.0))
            y = float(wp.get("y", 0.0))
            yaw = float(wp.get("yaw", 0.0))

            moved = self._move_to(x, y, yaw)
            if not moved:
                return ScanHomesResponse(success=False, message=f"failed to navigate to corner {i}")

            payload = ""
            for _ in range(self.scan_retries + 1):
                payload = self._scan_corner(remaining_payloads)
                if payload:
                    break

            if not payload:
                return ScanHomesResponse(success=False, message=f"failed to scan QR at corner {i}")

            if payload not in self.qr_expected_codes:
                return ScanHomesResponse(success=False, message=f"unexpected payload {payload}")

            color = self._payload_to_color(payload)
            if not color:
                return ScanHomesResponse(success=False, message=f"could not parse color from {payload}")
            if color in self.home_map:
                return ScanHomesResponse(success=False, message=f"duplicate color {color} detected")

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation = yaw_to_quat(yaw)
            self.home_map[color] = {"pose": pose, "payload": payload}
            remaining_payloads.discard(payload)

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
