#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
import threading

bridge = CvBridge()

COLOR_RANGES = {
    "red": [
        (np.array([0,   120, 70]),  np.array([10,  255, 255])),
        (np.array([170, 120, 70]),  np.array([180, 255, 255])),
    ],
    "green": [(np.array([40, 60, 60]),  np.array([85,  255, 255]))],
    "blue":  [(np.array([100, 80, 60]), np.array([130, 255, 255]))],
}
DRAW_COLORS = {"red": (0,0,255), "green": (0,255,0), "blue": (255,0,0)}
MIN_AREA = 300

last_vis = None
vis_lock = threading.Lock()


def get_depth_from_mask(depth_img, contour_mask):
    """
    Sample depth ONLY from pixels that belong to the puck mask.
    Ignores zeros (invalid/missing depth readings).
    Returns median depth in metres.
    """
    # Extract depth values only where the puck mask is white
    puck_depths = depth_img[contour_mask > 0].astype(np.float32)

    # Remove invalid zero readings (no depth data)
    puck_depths = puck_depths[puck_depths > 0]

    if puck_depths.size == 0:
        return None

    # Median is robust against edge noise and partial occlusions
    return float(np.median(puck_depths)) / 1000.0  # mm → metres


def callback(color_msg, depth_msg):
    global last_vis

    # --- Decode colour (compressed) ---
    arr   = np.frombuffer(color_msg.data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        return

    # --- Decode depth (raw 16-bit mm) ---
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    # Depth and colour may have different resolutions — resize depth to match colour
    if depth.shape[:2] != frame.shape[:2]:
        depth = cv2.resize(depth, (frame.shape[1], frame.shape[0]),
                           interpolation=cv2.INTER_NEAREST)  # NEAREST avoids blending depths

    vis = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name, ranges in COLOR_RANGES.items():

        # --- Build colour mask ---
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_AREA:
                continue

            # --- Build a mask for THIS specific contour only ---
            puck_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.drawContours(puck_mask, [cnt], -1, 255, thickness=cv2.FILLED)

            # --- Erode slightly to avoid sampling background at puck edges ---
            erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            puck_mask_eroded = cv2.erode(puck_mask, erode_kernel, iterations=1)

            # --- Get depth using ONLY puck pixels ---
            dist = get_depth_from_mask(depth, puck_mask_eroded)
            dist_str = f"{dist:.2f}m" if dist is not None else "N/A"

            # --- Draw results ---
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cx, cy, radius   = int(cx), int(cy), int(radius)
            col              = DRAW_COLORS[color_name]

            cv2.circle(vis, (cx, cy), radius, col, 2)
            cv2.circle(vis, (cx, cy), 4,      col, -1)

            # Draw the eroded mask outline so you can see what pixels are sampled
            contours_eroded, _ = cv2.findContours(puck_mask_eroded,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, contours_eroded, -1, (255, 255, 0), 1)

            label = f"{color_name}: {dist_str}"
            cv2.putText(vis, label,
                        (cx - radius, cy - radius - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

            rospy.loginfo_throttle(1, f"{color_name} puck at ({cx},{cy}): {dist_str}")

    with vis_lock:
        last_vis = vis


def main():
    rospy.init_node("detect_pucks_distance")

    color_sub = message_filters.Subscriber(
        "/camera/color/image_2fps/compressed", CompressedImage)
    depth_sub = message_filters.Subscriber(
        "/camera/depth/image_2fps", Image)

    sync = message_filters.ApproximateTimeSynchronizer(
        [color_sub, depth_sub], queue_size=5, slop=0.5)
    sync.registerCallback(callback)

    rospy.loginfo("detect_pucks_distance running — press 'q' to quit")

    while not rospy.is_shutdown():
        with vis_lock:
            frame = last_vis.copy() if last_vis is not None else None
        if frame is not None:
            cv2.imshow("detect_pucks_distance", frame)
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
