#!/usr/bin/env python3
"""
Proportional controller: keeps the red puck centred in the frame.
  - angular.z drives left/right to centre the puck horizontally
  - linear.x drives forward/back based on puck size (proxy for distance)
"""
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np

RED_RANGES = [
    (np.array([0,   120, 70]),  np.array([10,  255, 255])),
    (np.array([170, 120, 70]),  np.array([180, 255, 255])),
]
MIN_AREA        = 300    # px²
TARGET_RADIUS   = 40     # px  — desired apparent size (distance)
KP_ANGULAR      = 0.005  # gain for heading error
KP_LINEAR       = 0.003  # gain for distance error
MAX_LINEAR      = 0.15   # m/s
MAX_ANGULAR     = 0.8    # rad/s

pub  = None
last = None   # latest frame for display


def find_largest_red(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        if best is None or area > cv2.contourArea(best):
            best = cnt
    return best


def cb(msg):
    global last
    arr   = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        return

    h, w  = frame.shape[:2]
    cx_img = w // 2          # image centre x
    twist  = Twist()
    vis    = frame.copy()

    cnt = find_largest_red(frame)
    if cnt is not None:
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        cx, cy, radius   = int(cx), int(cy), int(radius)

        # Heading error: positive → puck is to the right → turn right (negative z)
        heading_err  = cx - cx_img
        # Distance error: positive → puck is too small → move forward
        distance_err = TARGET_RADIUS - radius

        twist.angular.z = float(np.clip(-KP_ANGULAR * heading_err,
                                        -MAX_ANGULAR, MAX_ANGULAR))
        twist.linear.x  = float(np.clip( KP_LINEAR  * distance_err,
                                        -MAX_LINEAR,  MAX_LINEAR))

        # Visualise
        cv2.circle(vis, (cx, cy), radius, (0, 0, 255), 2)
        cv2.line(vis,  (cx_img, 0), (cx_img, h), (255, 255, 0), 1)
        cv2.line(vis,  (cx_img, cy), (cx, cy),   (0,   255, 0), 2)
        cv2.putText(vis, f"err_h={heading_err:+d}  r={radius}px",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    else:
        # No puck — spin slowly to search
        twist.angular.z = 0.3
        cv2.putText(vis, "Searching...", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    pub.publish(twist)
    last = vis


def main():
    global pub
    rospy.init_node("follow_red_puck")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/camera/color/image_2fps/compressed",
                     CompressedImage, cb, queue_size=1)
    rospy.loginfo("follow_red_puck started — Ctrl+C to stop")

    while not rospy.is_shutdown():
        if last is not None:
            cv2.imshow("follow_red_puck", last)
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    pub.publish(Twist())   # stop robot on exit
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
