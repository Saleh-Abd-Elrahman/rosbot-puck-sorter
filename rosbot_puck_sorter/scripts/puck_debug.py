#!/usr/bin/env python3
"""
Debug viewer for the red puck detector. No motion, no gripper — just shows
the detected puck with its radius, centroid, and the gripper aim point.
Use it to read off the radius at "puck in gripper sweet spot" by pushing
the robot manually into position.
"""
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

RED_RANGES = [
    (np.array([0,   120, 70]),  np.array([10,  255, 255])),
    (np.array([170, 120, 70]),  np.array([180, 255, 255])),
]
MIN_AREA      = 300
AIM_OFFSET_PX = 30   # match grab_red_puck.py

last = None


def find_largest_red(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        if area > best_area:
            best, best_area = cnt, area
    return best, best_area


def cb(msg):
    global last
    arr   = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        return

    h, w   = frame.shape[:2]
    cx_img = w // 2
    aim_x  = cx_img + AIM_OFFSET_PX
    vis    = frame.copy()

    cnt, area = find_largest_red(frame)
    if cnt is not None:
        (fx, fy), fr = cv2.minEnclosingCircle(cnt)
        cx, cy, radius = int(fx), int(fy), int(fr)
        cv2.circle(vis, (cx, cy), radius, (0, 0, 255), 2)
        cv2.line(vis, (aim_x, cy), (cx, cy), (0, 255, 0), 2)
        text = f"r={radius}px  cx={cx}  cy={cy}  area={int(area)}  err_x={cx-aim_x:+d}"
        rospy.loginfo_throttle(1.0, text)
    else:
        text = "no puck"

    cv2.line(vis, (cx_img, 0), (cx_img, h), (120, 120, 0), 1)
    cv2.line(vis, (aim_x,  0), (aim_x,  h), (255, 255, 0), 1)
    cv2.putText(vis, text, (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    last = vis


def main():
    rospy.init_node("puck_debug")
    rospy.Subscriber("/camera/color/image_2fps/compressed",
                     CompressedImage, cb, queue_size=1)
    rospy.loginfo("puck_debug — q to quit")

    while not rospy.is_shutdown():
        if last is not None:
            cv2.imshow("puck_debug", last)
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
