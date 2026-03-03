#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def _load_aruco_color_map(raw_map):
    out = {}
    for k, v in raw_map.items():
        try:
            mk = int(k)
        except Exception:
            continue
        color = str(v).strip().lower()
        if color in ("red", "green", "blue"):
            out[mk] = color
    return out


class ArucoLiveCameraTest:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.aruco_dictionary_name = rospy.get_param("~aruco_dictionary", "DICT_4X4_50")
        self.aruco_id_to_color = _load_aruco_color_map(
            rospy.get_param("~aruco_id_to_color", {"0": "red", "1": "green", "2": "blue"})
        )
        self.expected_ids = [int(v) for v in rospy.get_param("~expected_ids", list(self.aruco_id_to_color.keys()))]
        self.require_all_expected = bool(rospy.get_param("~require_all_expected", True))
        self.stable_reads_required = int(rospy.get_param("~stable_reads_required", 8))
        self.timeout_s = float(rospy.get_param("~timeout_s", 30.0))
        self.min_perimeter_px = float(rospy.get_param("~min_perimeter_px", 80.0))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV aruco module is unavailable. Install opencv-contrib-python or ROS OpenCV with aruco.")

        dict_id = getattr(cv2.aruco, self.aruco_dictionary_name, None)
        if dict_id is None:
            raise RuntimeError(f"Unknown aruco dictionary: {self.aruco_dictionary_name}")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_detector = None

        self.bridge = CvBridge()
        self.debug_pub = rospy.Publisher("/aruco_test/debug_image", Image, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        self.last_seen_ids = set()
        self.stable_counts = {i: 0 for i in self.expected_ids}
        self.any_mapped_stable = 0

        rospy.loginfo("Aruco live test listening on %s", self.image_topic)
        rospy.loginfo("Expected ids=%s, stable_reads_required=%d, timeout=%.1fs", self.expected_ids, self.stable_reads_required, self.timeout_s)

    def _detect(self, frame):
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return [], []

        filtered_ids = []
        filtered_corners = []
        for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
            perimeter = cv2.arcLength(marker_corners.astype(np.float32), True)
            if perimeter < self.min_perimeter_px:
                continue
            filtered_ids.append(int(marker_id))
            filtered_corners.append(marker_corners)

        return filtered_ids, filtered_corners

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        ids, corners = self._detect(frame)
        seen_set = set(ids)
        self.last_seen_ids = seen_set

        if self.expected_ids:
            for marker_id in self.expected_ids:
                if marker_id in seen_set:
                    self.stable_counts[marker_id] += 1
                else:
                    self.stable_counts[marker_id] = 0
        else:
            mapped_seen = [i for i in seen_set if i in self.aruco_id_to_color]
            if mapped_seen:
                self.any_mapped_stable += 1
            else:
                self.any_mapped_stable = 0

        if self.publish_debug_image and self.debug_pub.get_num_connections() > 0:
            dbg = frame.copy()
            for marker_id, marker_corners in zip(ids, corners):
                pts = marker_corners.reshape(-1, 2).astype(int)
                cv2.polylines(dbg, [pts], isClosed=True, color=(0, 255, 255), thickness=2)
                c = pts.mean(axis=0).astype(int)
                color = self.aruco_id_to_color.get(int(marker_id), "unknown")
                cv2.putText(
                    dbg,
                    f"id={marker_id} {color}",
                    (int(c[0]), int(c[1])),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            dbg_msg.header = msg.header
            self.debug_pub.publish(dbg_msg)

    def _passed(self):
        if self.expected_ids:
            if self.require_all_expected:
                return all(self.stable_counts.get(i, 0) >= self.stable_reads_required for i in self.expected_ids)
            return any(self.stable_counts.get(i, 0) >= self.stable_reads_required for i in self.expected_ids)
        return self.any_mapped_stable >= self.stable_reads_required

    def run(self):
        start = time.time()
        last_log = 0.0
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            now = time.time()
            elapsed = now - start

            if self._passed():
                rospy.loginfo("PASS: ArUco live camera test")
                rospy.loginfo("Seen ids=%s stable_counts=%s", sorted(self.last_seen_ids), self.stable_counts)
                return

            if elapsed > self.timeout_s:
                raise RuntimeError(
                    "FAIL: timed out waiting for ArUco markers. "
                    f"last_seen_ids={sorted(self.last_seen_ids)} stable_counts={self.stable_counts}"
                )

            if now - last_log > 1.0:
                rospy.loginfo("Waiting... seen=%s stable=%s", sorted(self.last_seen_ids), self.stable_counts)
                last_log = now

            rate.sleep()


def main():
    rospy.init_node("test_aruco_live_camera")
    tester = ArucoLiveCameraTest()
    tester.run()
    print("PASS: test_aruco_live_camera")


if __name__ == "__main__":
    main()
