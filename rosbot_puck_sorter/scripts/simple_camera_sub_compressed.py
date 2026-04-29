#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def cb(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is not None:
        cv2.imshow("camera_2fps_compressed", img)
        cv2.waitKey(1)

rospy.init_node("camera_2fps_compressed_viewer")
rospy.Subscriber("/camera/color/image_2fps/compressed", CompressedImage, cb, queue_size=1)
rospy.spin()
