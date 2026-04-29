#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

# HSV ranges — tune these to your lighting conditions
COLOR_RANGES = {
    "red": [
        (np.array([0, 120, 70]),   np.array([10, 255, 255])),   # lower red
        (np.array([170, 120, 70]), np.array([180, 255, 255])),  # upper red (wraps hue)
    ],
    "green": [
        (np.array([40, 60, 60]),   np.array([85, 255, 255])),
    ],
    "blue": [
        (np.array([100, 80, 60]),  np.array([130, 255, 255])),
    ],
}

DRAW_COLORS = {
    "red":   (0, 0, 255),
    "green": (0, 255, 0),
    "blue":  (255, 0, 0),
}

MIN_AREA = 300  # px² — filter out noise


def detect_pucks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    vis = frame.copy()

    for color_name, ranges in COLOR_RANGES.items():
        # Build combined mask (handles red hue wrap-around)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lo, hi) in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cx, cy, radius = int(cx), int(cy), int(radius)

            draw_col = DRAW_COLORS[color_name]
            cv2.circle(vis, (cx, cy), radius, draw_col, 2)
            cv2.circle(vis, (cx, cy), 4,      draw_col, -1)
            cv2.putText(vis, f"{color_name} ({cx},{cy})",
                        (cx - radius, cy - radius - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_col, 1)

    return vis


def cb(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        return
    vis = detect_pucks(frame)
    cv2.imshow("detect_pucks", vis)
    cv2.waitKey(1)


rospy.init_node("detect_pucks")
rospy.Subscriber("/camera/color/image_2fps/compressed", CompressedImage, cb, queue_size=1)
rospy.spin()
cv2.destroyAllWindows()
