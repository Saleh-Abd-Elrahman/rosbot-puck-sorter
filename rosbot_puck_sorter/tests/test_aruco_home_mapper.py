#!/usr/bin/env python3
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from rosbot_puck_sorter.msg import HomeBaseArray
from rosbot_puck_sorter.srv import ScanHomes

from common import load_script_module, safe_shutdown, wait_for


class FakeNavigator:
    def __init__(self):
        self.goals = []

    def goto(self, x, y, yaw, timeout_s=45.0):
        self.goals.append((x, y, yaw, timeout_s))
        return True


def generate_marker(dictionary, marker_id, size_px=160):
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    else:
        marker = np.zeros((size_px, size_px), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, size_px, marker, 1)
    return marker


def main():
    rospy.init_node("test_aruco_home_mapper")

    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco module is required for test_aruco_home_mapper")

    rospy.set_param("~corner_waypoints", [
        {"x": 0.2, "y": 0.2, "yaw": 0.0},
        {"x": 0.8, "y": 0.2, "yaw": 1.57},
        {"x": 0.2, "y": 0.8, "yaw": -1.57},
    ])
    rospy.set_param("~marker_mode", "aruco")
    rospy.set_param("~aruco_dictionary", "DICT_4X4_50")
    rospy.set_param("~aruco_id_to_color", {"0": "red", "1": "green", "2": "blue"})
    rospy.set_param("~scan_timeout_s", 2.0)
    rospy.set_param("~scan_retries", 1)
    rospy.set_param("~stable_reads_required", 2)
    rospy.set_param("~image_topic", "/camera/color/image_raw")
    rospy.set_param("~publish_start_relative", False)

    module = load_script_module("qr_home_mapper.py", "qr_home_mapper_test_mod")
    module.SimplePoseNavigator = FakeNavigator
    module.QRHomeMapper()

    bridge = CvBridge()
    image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas = np.full((600, 800, 3), 255, dtype=np.uint8)
    marker_ids = [0, 1, 2]
    positions = [(80, 80), (320, 220), (560, 360)]
    for marker_id, (x, y) in zip(marker_ids, positions):
        m = generate_marker(dictionary, marker_id, size_px=140)
        canvas[y : y + 140, x : x + 140, :] = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)

    stop_flag = {"stop": False}

    def publisher_loop():
        rate = rospy.Rate(15)
        while not stop_flag["stop"] and not rospy.is_shutdown():
            msg = bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera_color_optical_frame"
            image_pub.publish(msg)
            rate.sleep()

    pub_thread = threading.Thread(target=publisher_loop, daemon=True)
    pub_thread.start()

    homes_msg = {"data": None}

    def homes_cb(msg):
        homes_msg["data"] = msg

    rospy.Subscriber("/home_bases", HomeBaseArray, homes_cb, queue_size=1)

    rospy.wait_for_service("/scan_homes", timeout=5.0)
    scan_srv = rospy.ServiceProxy("/scan_homes", ScanHomes)

    rsp = scan_srv(start=True)
    if not rsp.success:
        raise RuntimeError(f"scan_homes failed: {rsp.message}")

    wait_for(lambda: homes_msg["data"] is not None, timeout_s=3.0, desc="/home_bases message")

    homes = homes_msg["data"].homes
    colors = set([h.color for h in homes])
    if colors != {"red", "green", "blue"}:
        raise RuntimeError(f"unexpected home colors: {colors}")

    if len(homes) != 3:
        raise RuntimeError(f"expected 3 homes, got {len(homes)}")

    stop_flag["stop"] = True
    time.sleep(0.1)
    print("PASS: ArUco home mapping test")
    safe_shutdown()


if __name__ == "__main__":
    main()
