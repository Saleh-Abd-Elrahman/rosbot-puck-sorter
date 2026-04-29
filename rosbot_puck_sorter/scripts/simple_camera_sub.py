#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def cb(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imshow("camera_2fps", img)
    cv2.waitKey(1)

rospy.init_node("camera_2fps_viewer")
rospy.Subscriber("/camera/color/image_2fps", Image, cb, queue_size=1)
rospy.spin()
