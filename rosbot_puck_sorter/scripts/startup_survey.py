#!/usr/bin/env python3
import json
import math
import os
import threading
import time
from collections import Counter

import cv2
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion, Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger, TriggerResponse

from rosbot_puck_sorter.msg import HomeBase, HomeBaseArray, PuckDetectionArray, PuckTrack, PuckTrackArray


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class StartupSurvey:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.start_frame = rospy.get_param("~start_frame", "start")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.puck_detection_topic = rospy.get_param("~puck_detection_topic", "/puck/detections")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

        self.marker_mode = str(rospy.get_param("~marker_mode", "aruco")).strip().lower()
        self.aruco_dictionary_name = rospy.get_param("~aruco_dictionary", "DICT_4X4_50")
        self.aruco_id_to_color = self._load_aruco_color_map(
            rospy.get_param("~aruco_id_to_color", {"0": "red", "1": "green", "2": "blue"})
        )
        self.marker_size_m = float(rospy.get_param("~marker_size_m", 0.05))
        self.min_aruco_perimeter_px = float(rospy.get_param("~min_aruco_perimeter_px", 80.0))
        self.max_marker_distance_m = float(rospy.get_param("~max_marker_distance_m", 0.0))

        self.use_camera_info_intrinsics = bool(rospy.get_param("~use_camera_info_intrinsics", True))
        self.fx_override = float(rospy.get_param("~fx", 0.0))
        self.fy_override = float(rospy.get_param("~fy", 0.0))
        self.cx_override = float(rospy.get_param("~cx", 0.0))
        self.cy_override = float(rospy.get_param("~cy", 0.0))
        self.hfov_deg = float(rospy.get_param("~hfov_deg", 70.0))

        self.rotation_rad = float(rospy.get_param("~rotation_rad", 6.35))
        self.rotation_speed_rad_s = float(rospy.get_param("~rotation_speed_rad_s", 0.45))
        self.cmd_rate_hz = float(rospy.get_param("~cmd_rate_hz", 20.0))
        self.max_duration_s = float(rospy.get_param("~max_duration_s", 30.0))
        self.stable_reads_required = int(rospy.get_param("~stable_reads_required", 3))
        self.min_home_colors_required = int(rospy.get_param("~min_home_colors_required", 3))

        self.puck_conf_min = float(rospy.get_param("~puck_conf_min", 0.45))
        self.puck_cluster_radius_m = float(rospy.get_param("~puck_cluster_radius_m", 0.15))

        self.require_start_frame_initialized = bool(rospy.get_param("~require_start_frame_initialized", True))
        self.start_frame_wait_s = float(rospy.get_param("~start_frame_wait_s", 8.0))

        self.snapshot_path = os.path.expanduser(
            rospy.get_param("~snapshot_path", "~/.ros/rosbot_puck_sorter_startup_survey.json")
        )

        self.latest_bgr = None
        self.latest_stamp = rospy.Time(0)
        self.latest_frame_id = ""
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.start_frame_initialized = False
        self.survey_active = False

        self.home_obs = {"red": [], "green": [], "blue": []}
        self.puck_obs = []

        self.aruco_dict = None
        self.aruco_detector = None
        self.aruco_params = None
        self._init_aruco_detector()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.pub_homes = rospy.Publisher("/startup_survey/homes_start", HomeBaseArray, queue_size=1, latch=True)
        self.pub_pucks = rospy.Publisher("/startup_survey/pucks_start", PuckTrackArray, queue_size=1, latch=True)

        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber(self.puck_detection_topic, PuckDetectionArray, self._puck_cb, queue_size=30)
        rospy.Subscriber("/start_frame/initialized", Bool, self._start_init_cb, queue_size=1)

        self.service = rospy.Service("/startup_survey/run", Trigger, self._run_cb)

        rospy.loginfo("startup_survey ready")

    def _load_aruco_color_map(self, raw_map):
        out = {}
        for k, v in raw_map.items():
            try:
                mid = int(k)
            except Exception:
                continue
            color = str(v).strip().lower()
            if color in ("red", "green", "blue"):
                out[mid] = color
        return out

    def _init_aruco_detector(self):
        if self.marker_mode != "aruco":
            return
        if not hasattr(cv2, "aruco"):
            rospy.logerr("startup_survey requires OpenCV aruco module")
            return

        dict_id = getattr(cv2.aruco, self.aruco_dictionary_name, None)
        if dict_id is None:
            rospy.logwarn("unknown aruco dictionary %s, falling back to DICT_4X4_50", self.aruco_dictionary_name)
            dict_id = cv2.aruco.DICT_4X4_50

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def _start_init_cb(self, msg):
        self.start_frame_initialized = bool(msg.data)

    def _image_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self.lock:
            self.latest_bgr = bgr
            self.latest_stamp = msg.header.stamp
            self.latest_frame_id = msg.header.frame_id

    def _camera_info_cb(self, msg):
        if not self.use_camera_info_intrinsics:
            return
        self.fx = float(msg.K[0])
        self.fy = float(msg.K[4])
        self.cx = float(msg.K[2])
        self.cy = float(msg.K[5])

    def _camera_matrix(self, width, height):
        if self.fx_override > 0.0 and self.fy_override > 0.0:
            fx = self.fx_override
            fy = self.fy_override
            cx = self.cx_override if self.cx_override > 0.0 else (width / 2.0)
            cy = self.cy_override if self.cy_override > 0.0 else (height / 2.0)
        elif self.use_camera_info_intrinsics and self.fx and self.fy:
            fx = self.fx
            fy = self.fy
            cx = self.cx if self.cx is not None else (width / 2.0)
            cy = self.cy if self.cy is not None else (height / 2.0)
        else:
            hfov_rad = self.hfov_deg * np.pi / 180.0
            fx = (width / 2.0) / max(1e-6, np.tan(hfov_rad / 2.0))
            fy = fx
            cx = width / 2.0
            cy = height / 2.0

        k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
        d = np.zeros((5, 1), dtype=np.float32)
        return k, d

    def _estimate_tvec(self, marker_corners, k, d):
        corners = marker_corners.astype(np.float32).reshape(1, 4, 2)
        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            _r, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size_m, k, d)
            if tvecs is None or len(tvecs) == 0:
                return None
            return tvecs[0][0]

        half = self.marker_size_m / 2.0
        obj = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )
        img = marker_corners.reshape(4, 2).astype(np.float32)
        ok, _rv, tv = cv2.solvePnP(obj, img, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        return tv.reshape(3)

    def _transform_point(self, point_xyz, src_frame, stamp, target_frame):
        ps = PointStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = src_frame
        ps.point = Point(x=float(point_xyz[0]), y=float(point_xyz[1]), z=float(point_xyz[2]))
        try:
            out = self.tf_buffer.transform(ps, target_frame, timeout=rospy.Duration(0.05))
            return out.point
        except Exception:
            return None

    def _collect_aruco(self):
        if self.marker_mode != "aruco" or self.aruco_dict is None:
            return

        with self.lock:
            if self.latest_bgr is None:
                return
            frame = self.latest_bgr.copy()
            stamp = self.latest_stamp
            frame_id = self.latest_frame_id

        if not frame_id:
            return

        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        h, w = frame.shape[:2]
        k, d = self._camera_matrix(w, h)

        for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
            marker_id = int(marker_id)
            color = self.aruco_id_to_color.get(marker_id)
            if not color:
                continue

            perimeter = cv2.arcLength(marker_corners.astype(np.float32), True)
            if perimeter < self.min_aruco_perimeter_px:
                continue

            tvec = self._estimate_tvec(marker_corners, k, d)
            if tvec is None:
                continue

            distance_m = float(np.linalg.norm(tvec))
            if self.max_marker_distance_m > 0.0 and distance_m > self.max_marker_distance_m:
                continue

            p_start = self._transform_point(tvec, frame_id, stamp, self.start_frame)
            if p_start is None:
                continue

            obs = {
                "payload": f"ARUCO_{marker_id}",
                "distance_m": distance_m,
                "x": float(p_start.x),
                "y": float(p_start.y),
                "z": float(p_start.z),
                "stamp": float(stamp.to_sec()),
            }
            self.home_obs[color].append(obs)

    def _puck_cb(self, msg):
        if not self.survey_active:
            return

        for det in msg.detections:
            if det.confidence < self.puck_conf_min:
                continue

            p_start = self._transform_point(
                [det.position_camera.x, det.position_camera.y, det.position_camera.z],
                msg.header.frame_id,
                msg.header.stamp,
                self.start_frame,
            )
            if p_start is None:
                continue

            self.puck_obs.append(
                {
                    "color": det.color,
                    "x": float(p_start.x),
                    "y": float(p_start.y),
                    "z": float(p_start.z),
                    "confidence": float(det.confidence),
                }
            )

    def _cluster_pucks(self):
        clusters = []
        r2 = self.puck_cluster_radius_m * self.puck_cluster_radius_m

        for obs in self.puck_obs:
            matched = None
            for c in clusters:
                if c["color"] != obs["color"]:
                    continue
                dx = obs["x"] - c["x"]
                dy = obs["y"] - c["y"]
                if (dx * dx + dy * dy) <= r2:
                    matched = c
                    break

            if matched is None:
                clusters.append(
                    {
                        "color": obs["color"],
                        "x": obs["x"],
                        "y": obs["y"],
                        "z": obs["z"],
                        "count": 1,
                        "conf_sum": obs["confidence"],
                    }
                )
            else:
                n = matched["count"]
                matched["x"] = (matched["x"] * n + obs["x"]) / (n + 1)
                matched["y"] = (matched["y"] * n + obs["y"]) / (n + 1)
                matched["z"] = (matched["z"] * n + obs["z"]) / (n + 1)
                matched["count"] = n + 1
                matched["conf_sum"] += obs["confidence"]

        return clusters

    def _summarize_homes(self):
        msg = HomeBaseArray()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.start_frame)

        summary = {}
        for color in ["red", "green", "blue"]:
            obs = self.home_obs[color]
            if len(obs) < self.stable_reads_required:
                continue

            xs = np.array([o["x"] for o in obs], dtype=np.float32)
            ys = np.array([o["y"] for o in obs], dtype=np.float32)
            zs = np.array([o["z"] for o in obs], dtype=np.float32)
            ds = np.array([o["distance_m"] for o in obs], dtype=np.float32)
            payload_mode = Counter([o["payload"] for o in obs]).most_common(1)[0][0]

            h = HomeBase()
            h.header = msg.header
            h.color = color
            h.pose_map = Pose()
            h.pose_map.position.x = float(np.median(xs))
            h.pose_map.position.y = float(np.median(ys))
            h.pose_map.position.z = float(np.median(zs))
            h.pose_map.orientation = yaw_to_quat(0.0)
            h.qr_payload = payload_mode
            h.marker_distance_m = float(np.median(ds))
            msg.homes.append(h)

            summary[color] = {
                "x": float(h.pose_map.position.x),
                "y": float(h.pose_map.position.y),
                "z": float(h.pose_map.position.z),
                "distance_m": float(h.marker_distance_m),
                "payload": payload_mode,
                "samples": len(obs),
            }

        return msg, summary

    def _summarize_pucks(self):
        clusters = self._cluster_pucks()
        msg = PuckTrackArray()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.start_frame)

        summary = []
        next_id = 1
        for c in clusters:
            tr = PuckTrack()
            tr.header = msg.header
            tr.track_id = next_id
            next_id += 1
            tr.color = c["color"]
            tr.pose_map = Pose()
            tr.pose_map.position.x = c["x"]
            tr.pose_map.position.y = c["y"]
            tr.pose_map.position.z = c["z"]
            tr.pose_map.orientation = yaw_to_quat(0.0)
            tr.confidence = float(c["conf_sum"] / max(1, c["count"]))
            tr.state = "DETECTED"
            tr.miss_count = 0
            tr.last_seen = rospy.Time.now()
            msg.tracks.append(tr)

            summary.append(
                {
                    "track_id": tr.track_id,
                    "color": tr.color,
                    "x": float(tr.pose_map.position.x),
                    "y": float(tr.pose_map.position.y),
                    "z": float(tr.pose_map.position.z),
                    "confidence": float(tr.confidence),
                    "samples": int(c["count"]),
                }
            )

        return msg, summary

    def _write_snapshot(self, homes_summary, pucks_summary):
        payload = {
            "timestamp": time.time(),
            "frame": self.start_frame,
            "homes": homes_summary,
            "pucks": pucks_summary,
        }

        path = self.snapshot_path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)

    def _wait_start_frame(self):
        if not self.require_start_frame_initialized:
            return True

        deadline = time.time() + self.start_frame_wait_s
        while time.time() < deadline and not rospy.is_shutdown():
            if self.start_frame_initialized:
                return True
            rospy.sleep(0.05)

        return False

    def _run_survey(self):
        if self.survey_active:
            return False, "survey already running"

        if not self._wait_start_frame():
            return False, "start frame is not initialized"

        self.home_obs = {"red": [], "green": [], "blue": []}
        self.puck_obs = []

        self.survey_active = True
        angle_acc = 0.0
        start_t = time.time()
        last_t = start_t
        rate = rospy.Rate(max(1.0, self.cmd_rate_hz))

        try:
            while not rospy.is_shutdown():
                now_t = time.time()
                dt = max(0.0, now_t - last_t)
                last_t = now_t

                if (now_t - start_t) > self.max_duration_s:
                    break

                if angle_acc >= self.rotation_rad:
                    break

                cmd = Twist()
                cmd.angular.z = self.rotation_speed_rad_s
                self.pub_cmd.publish(cmd)

                angle_acc += abs(self.rotation_speed_rad_s) * dt
                self._collect_aruco()
                rate.sleep()
        finally:
            self.survey_active = False
            self.pub_cmd.publish(Twist())

        homes_msg, homes_summary = self._summarize_homes()
        pucks_msg, pucks_summary = self._summarize_pucks()

        self.pub_homes.publish(homes_msg)
        self.pub_pucks.publish(pucks_msg)

        try:
            self._write_snapshot(homes_summary, pucks_summary)
        except Exception as exc:
            rospy.logwarn("startup survey snapshot write failed: %s", exc)

        detected_home_colors = len(homes_summary.keys())
        success = detected_home_colors >= self.min_home_colors_required
        summary = (
            f"survey done: homes={detected_home_colors}/{self.min_home_colors_required} "
            f"pucks={len(pucks_summary)} snapshot={self.snapshot_path}"
        )

        return success, summary

    def _run_cb(self, _req):
        ok, msg = self._run_survey()
        return TriggerResponse(success=ok, message=msg)


def main():
    rospy.init_node("startup_survey")
    StartupSurvey()
    rospy.spin()


if __name__ == "__main__":
    main()
