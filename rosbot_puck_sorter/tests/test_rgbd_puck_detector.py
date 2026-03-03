#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from rosbot_puck_sorter.msg import PuckDetectionArray

from common import load_script_module, safe_shutdown, wait_for


def main():
    rospy.init_node("test_rgbd_puck_detector")

    rospy.set_param("~map_frame", "camera_color_optical_frame")
    rospy.set_param("~image_topic", "/camera/color/image_raw")
    rospy.set_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
    rospy.set_param("~camera_info_topic", "/camera/color/camera_info")
    rospy.set_param("~roi_bottom_crop", 0.0)
    rospy.set_param("~publish_debug_image", False)
    rospy.set_param("~detection_conf_min", 0.4)

    module = load_script_module("rgbd_puck_detector.py", "rgbd_puck_detector_test_mod")
    module.RGBDPuckDetector()

    bridge = CvBridge()
    pub_rgb = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
    pub_depth = rospy.Publisher("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1)
    pub_info = rospy.Publisher("/camera/color/camera_info", CameraInfo, queue_size=1, latch=True)

    detections = {"msg": None}

    def det_cb(msg):
        detections["msg"] = msg

    rospy.Subscriber("/puck/detections", PuckDetectionArray, det_cb, queue_size=10)

    cam_info = CameraInfo()
    cam_info.header.frame_id = "camera_color_optical_frame"
    cam_info.width = 640
    cam_info.height = 480
    cam_info.K = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    pub_info.publish(cam_info)

    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(rgb, (160, 200), 35, (0, 0, 255), -1)   # red
    cv2.circle(rgb, (320, 200), 35, (0, 255, 0), -1)   # green
    cv2.circle(rgb, (480, 200), 35, (255, 0, 0), -1)   # blue

    depth = np.full((480, 640), 1.0, dtype=np.float32)

    rate = rospy.Rate(12)
    for _ in range(40):
        stamp = rospy.Time.now()

        rgb_msg = bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "camera_color_optical_frame"

        depth_msg = bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_color_optical_frame"

        pub_rgb.publish(rgb_msg)
        pub_depth.publish(depth_msg)
        rate.sleep()

    wait_for(lambda: detections["msg"] is not None and len(detections["msg"].detections) > 0, timeout_s=3.0, desc="puck detections")

    colors = set([d.color for d in detections["msg"].detections])
    missing = {"red", "green", "blue"} - colors
    if missing:
        raise RuntimeError(f"missing detections for colors: {missing}; got={colors}")

    time.sleep(0.1)
    print("PASS: RGB-D puck detector color test")
    safe_shutdown()


if __name__ == "__main__":
    main()
