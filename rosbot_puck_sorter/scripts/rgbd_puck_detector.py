#!/usr/bin/env python3
import math
import threading

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from rosbot_puck_sorter.msg import PuckDetection, PuckDetectionArray


class RGBDPuckDetector:
    def __init__(self):
        self.bridge = CvBridge()

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.image_is_compressed = bool(rospy.get_param("~image_is_compressed", self.image_topic.endswith("/compressed")))
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.process_rate_hz = float(rospy.get_param("~process_rate_hz", 12.0))
        self.require_map_tf = bool(rospy.get_param("~require_map_tf", False))
        self.allow_camera_info_fallback = bool(rospy.get_param("~allow_camera_info_fallback", True))
        self.hfov_deg = float(rospy.get_param("~hfov_deg", 70.0))
        self.debug_log_stats = bool(rospy.get_param("~debug_log_stats", True))

        self.depth_min_m = rospy.get_param("~depth_min_m", 0.15)
        self.depth_max_m = rospy.get_param("~depth_max_m", 1.80)
        self.roi_bottom_crop = rospy.get_param("~roi_bottom_crop", 0.15)
        self.median_blur_ksize = int(rospy.get_param("~median_blur_ksize", 5))
        self.morph_open_ksize = int(rospy.get_param("~morph_open_ksize", 3))
        self.morph_close_ksize = int(rospy.get_param("~morph_close_ksize", 5))
        self.morph_ksize = int(rospy.get_param("~morph_ksize", 7))
        self.depth_erode_ksize = int(rospy.get_param("~depth_erode_ksize", 5))
        self.min_area_px = int(rospy.get_param("~min_area_px", 120))
        self.max_area_px = int(rospy.get_param("~max_area_px", 20000))
        self.circularity_min = rospy.get_param("~circularity_min", 0.60)
        self.detection_conf_min = rospy.get_param("~detection_conf_min", 0.50)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)

        self.red1 = rospy.get_param("~red1", [0, 10, 90, 255, 70, 255])
        self.red2 = rospy.get_param("~red2", [170, 179, 90, 255, 70, 255])
        self.green = rospy.get_param("~green", [40, 85, 70, 255, 60, 255])
        self.blue = rospy.get_param("~blue", [95, 130, 70, 255, 60, 255])

        self.camera_lock = threading.Lock()
        self.fx = self.fy = self.cx = self.cy = None
        self.last_process_time = rospy.Time(0)
        self.last_stats_log_time = rospy.Time(0)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_detections = rospy.Publisher("/puck/detections", PuckDetectionArray, queue_size=10)
        self.pub_debug = rospy.Publisher("/puck/debug_image", Image, queue_size=1)

        self.sub_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)

        rgb_msg_type = CompressedImage if self.image_is_compressed else Image
        rgb_sub = message_filters.Subscriber(self.image_topic, rgb_msg_type)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.5)
        self.sync.registerCallback(self._rgbd_cb)

        rospy.loginfo("rgbd_puck_detector ready (image_topic=%s compressed=%s)", self.image_topic, self.image_is_compressed)

    def _camera_info_cb(self, msg):
        with self.camera_lock:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]

    def _hsv_mask(self, hsv, color_name):
        def bounds(spec):
            # Config format: [h_min, h_max, s_min, s_max, v_min, v_max].
            return np.array([spec[0], spec[2], spec[4]]), np.array([spec[1], spec[3], spec[5]])

        if color_name == "red":
            red1_low, red1_high = bounds(self.red1)
            red2_low, red2_high = bounds(self.red2)
            m1 = cv2.inRange(hsv, red1_low, red1_high)
            m2 = cv2.inRange(hsv, red2_low, red2_high)
            mask = cv2.bitwise_or(m1, m2)
        elif color_name == "green":
            low, high = bounds(self.green)
            mask = cv2.inRange(hsv, low, high)
        else:
            low, high = bounds(self.blue)
            mask = cv2.inRange(hsv, low, high)

        if self.median_blur_ksize >= 3 and self.median_blur_ksize % 2 == 1:
            mask = cv2.medianBlur(mask, self.median_blur_ksize)

        if self.morph_ksize >= 3:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_ksize, self.morph_ksize))
            kernel_open = kernel_close = kernel
        else:
            kernel_open = np.ones((self.morph_open_ksize, self.morph_open_ksize), np.uint8)
            kernel_close = np.ones((self.morph_close_ksize, self.morph_close_ksize), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        return mask

    @staticmethod
    def _contour_circularity(contour):
        area = cv2.contourArea(contour)
        peri = cv2.arcLength(contour, True)
        if peri <= 1e-6:
            return 0.0
        return 4.0 * math.pi * area / (peri * peri)

    @staticmethod
    def _median_patch(depth_img, u, v, size=5):
        h, w = depth_img.shape[:2]
        x0 = max(0, u - size)
        x1 = min(w, u + size + 1)
        y0 = max(0, v - size)
        y1 = min(h, v + size + 1)
        patch = depth_img[y0:y1, x0:x1]
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0.0]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def _depth_from_contour(self, depth_img, contour, image_shape):
        mask = np.zeros(image_shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        if self.depth_erode_ksize >= 3:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.depth_erode_ksize, self.depth_erode_ksize))
            mask = cv2.erode(mask, kernel, iterations=1)
        values = depth_img[mask > 0].astype(np.float32)
        values = values[np.isfinite(values)]
        values = values[values > 0.0]
        if values.size == 0:
            return None
        return float(np.median(values))

    def _bgr_from_msg(self, msg):
        if isinstance(msg, CompressedImage):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("compressed image decode returned None")
            return frame
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _pixel_to_camera(self, u, v, depth_m, image_w, image_h):
        with self.camera_lock:
            fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        if any(value is None for value in [fx, fy, cx, cy]):
            if not self.allow_camera_info_fallback:
                return None
            hfov_rad = math.radians(self.hfov_deg)
            fx = (float(image_w) / 2.0) / max(1e-6, math.tan(hfov_rad / 2.0))
            fy = fx
            cx = float(image_w) / 2.0
            cy = float(image_h) / 2.0
        if fx <= 0.0 or fy <= 0.0:
            return None
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return x, y, z

    def _to_map(self, cam_frame, x, y, z, stamp):
        if cam_frame == self.map_frame:
            return Point(x=x, y=y, z=z)

        ps = PointStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = cam_frame
        ps.point = Point(x=x, y=y, z=z)

        try:
            out = self.tf_buffer.transform(ps, self.map_frame, timeout=rospy.Duration(0.08))
            return out.point
        except Exception as exc:
            if self.require_map_tf:
                rospy.logwarn_throttle(2.0, "skipping puck detection; no TF %s -> %s: %s", cam_frame, self.map_frame, exc)
            return None

    def _rgbd_cb(self, rgb_msg, depth_msg):
        if self.process_rate_hz > 0.0:
            now = rospy.Time.now()
            min_period = 1.0 / self.process_rate_hz
            if self.last_process_time != rospy.Time(0) and (now - self.last_process_time).to_sec() < min_period:
                return
            self.last_process_time = now

        try:
            bgr = self._bgr_from_msg(rgb_msg)
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        if depth.dtype != np.float32:
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)

        h, w = bgr.shape[:2]
        if depth.shape[:2] != bgr.shape[:2]:
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
        roi_h = int(h * (1.0 - self.roi_bottom_crop))
        roi = bgr[0:roi_h, :]
        depth_roi = depth[0:roi_h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        det_array = PuckDetectionArray()
        det_array.header = rgb_msg.header

        debug = roi.copy()
        stats = {}

        for color in ["red", "green", "blue"]:
            mask = self._hsv_mask(hsv, color)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            stats[color] = {
                "contours": len(contours),
                "area": 0,
                "circular": 0,
                "depth": 0,
                "camera": 0,
                "detections": 0,
            }

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area_px or area > self.max_area_px:
                    continue
                stats[color]["area"] += 1

                circ = self._contour_circularity(contour)
                if circ < self.circularity_min:
                    continue
                stats[color]["circular"] += 1

                (u_f, v_f), radius_px = cv2.minEnclosingCircle(contour)
                u = int(u_f)
                v = int(v_f)

                depth_m = self._depth_from_contour(depth_roi, contour, roi.shape)
                if depth_m is None:
                    depth_m = self._median_patch(depth_roi, u, v, size=5)
                if depth_m is None:
                    continue
                if depth_m < self.depth_min_m or depth_m > self.depth_max_m:
                    continue
                stats[color]["depth"] += 1

                xyz = self._pixel_to_camera(u, v, depth_m, w, h)
                if xyz is None:
                    continue
                stats[color]["camera"] += 1
                x_cam, y_cam, z_cam = xyz

                conf = min(0.99, max(0.0, 0.45 + 0.35 * circ + 0.20 * min(1.0, area / 1000.0)))
                if conf < self.detection_conf_min:
                    continue
                stats[color]["detections"] += 1

                point_map = self._to_map(rgb_msg.header.frame_id, x_cam, y_cam, z_cam, rgb_msg.header.stamp)
                if point_map is None:
                    if self.require_map_tf:
                        continue
                    point_map = Point(x=x_cam, y=y_cam, z=z_cam)

                det = PuckDetection()
                det.header = rgb_msg.header
                det.color = color
                det.position_camera = Point(x=x_cam, y=y_cam, z=z_cam)
                det.position_map = point_map
                det.confidence = conf
                det.radius_m = float(radius_px * depth_m / max(1e-6, self.fx if self.fx else 1.0))
                det.contour_area_px = int(area)
                det_array.detections.append(det)

                if self.publish_debug_image:
                    cv2.circle(debug, (u, v), int(radius_px), (0, 255, 255), 2)
                    cv2.putText(
                        debug,
                        f"{color}:{conf:.2f}",
                        (u + 6, max(15, v - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

        self.pub_detections.publish(det_array)
        self._log_stats(stats, len(det_array.detections))

        if self.publish_debug_image and self.pub_debug.get_num_connections() > 0:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            dbg_msg.header = rgb_msg.header
            self.pub_debug.publish(dbg_msg)

    def _log_stats(self, stats, detections):
        if not self.debug_log_stats:
            return
        now = rospy.Time.now()
        if self.last_stats_log_time != rospy.Time(0) and (now - self.last_stats_log_time).to_sec() < 2.0:
            return
        self.last_stats_log_time = now
        parts = []
        for color in ["red", "green", "blue"]:
            s = stats.get(color, {})
            parts.append(
                "%s c=%d area=%d circ=%d depth=%d cam=%d det=%d"
                % (
                    color,
                    s.get("contours", 0),
                    s.get("area", 0),
                    s.get("circular", 0),
                    s.get("depth", 0),
                    s.get("camera", 0),
                    s.get("detections", 0),
                )
            )
        rospy.loginfo("puck detector stats: total=%d | %s", detections, " | ".join(parts))


def main():
    rospy.init_node("rgbd_puck_detector")
    RGBDPuckDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
