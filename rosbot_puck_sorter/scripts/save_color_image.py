#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading

image_count = 0
current_img = None
img_lock = threading.Lock()  # protect shared variable

def cb(msg):
    global current_img
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is not None:
        with img_lock:
            current_img = frame  # just update, never call imshow here

def main():
    global image_count, current_img

    rospy.init_node("save_color_image")
    rospy.Subscriber("/camera/color/image_2fps/compressed",
                     CompressedImage, cb, queue_size=1)

    rospy.loginfo("Press 's' to save, 'q' to quit")

    while not rospy.is_shutdown():
        with img_lock:
            frame = current_img.copy() if current_img is not None else None

        if frame is not None:
            cv2.imshow("camera", frame)

        key = cv2.waitKey(50) & 0xFF  # all GUI calls in main thread only

        if key == ord('s') and frame is not None:
            filename = f"puck_{image_count:03d}.png"
            cv2.imwrite(filename, frame)
            rospy.loginfo(f"Saved {filename}")
            image_count += 1

        elif key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
