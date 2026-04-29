#!/usr/bin/env python3
import math
import threading
from dataclasses import dataclass

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import Bool

from rosbot_puck_sorter.msg import MissionState, PuckDetectionArray
from rosbot_puck_sorter.srv import SetGripper


@dataclass
class MarkerObservation:
    marker_id: int
    color: str
    x: float
    z: float
    stamp: rospy.Time
    distance_m: float
    size_px: float


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class ChallengeManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.image_is_compressed = bool(rospy.get_param("~image_is_compressed", self.image_topic.endswith("/compressed")))
        self.raw_image_fallback_topic = rospy.get_param("~raw_image_fallback_topic", "")
        if not self.raw_image_fallback_topic and self.image_is_compressed and self.image_topic.endswith("/compressed"):
            self.raw_image_fallback_topic = self.image_topic[: -len("/compressed")]
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.gripper_service = rospy.get_param("~gripper_service", "/gripper/set")
        self.gripper_holding_topic = rospy.get_param("~gripper_holding_topic", "/gripper/holding_object")

        self.expected_colors = self._normalize_color_list(rospy.get_param("~expected_colors", ["red", "green", "blue"]))
        self.marker_id_to_color = self._load_marker_map(rospy.get_param("~marker_id_to_color", {"1": "red", "2": "green", "3": "blue"}))

        self.aruco_dictionary_name = rospy.get_param("~aruco_dictionary", "DICT_4X4_50")
        self.marker_size_m = float(rospy.get_param("~marker_size_m", 0.05))
        self.use_camera_info_intrinsics = bool(rospy.get_param("~use_camera_info_intrinsics", True))
        self.hfov_deg = float(rospy.get_param("~hfov_deg", 70.0))
        self.min_aruco_perimeter_px = float(rospy.get_param("~min_aruco_perimeter_px", 70.0))
        self.marker_stale_s = float(rospy.get_param("~marker_stale_s", 0.5))
        self.puck_stale_s = float(rospy.get_param("~puck_stale_s", 0.5))
        self.puck_conf_min = float(rospy.get_param("~puck_conf_min", 0.45))

        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 20.0))
        self.search_angular_speed = float(rospy.get_param("~search_angular_speed_rad_s", 0.35))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed_m_s", 0.10))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed_rad_s", 0.55))
        self.kp_yaw = float(rospy.get_param("~kp_yaw", 1.8))
        self.kp_depth = float(rospy.get_param("~kp_depth", 0.8))
        self.lateral_deadband_m = float(rospy.get_param("~lateral_deadband_m", 0.025))
        self.depth_deadband_m = float(rospy.get_param("~depth_deadband_m", 0.025))
        self.align_hold_s = float(rospy.get_param("~align_hold_s", 0.25))

        self.gripper_aim_offset_px = float(rospy.get_param("~gripper_aim_offset_px", 50.0))
        self.pickup_target_depth_m = float(rospy.get_param("~pickup_target_depth_m", 0.24))
        self.approach_base_speed = float(rospy.get_param("~approach_base_speed_m_s", 0.04))
        self.pickup_commit_enabled = bool(rospy.get_param("~pickup_commit_enabled", True))
        self.pickup_commit_radius_px = float(rospy.get_param("~pickup_commit_radius_px", 45.0))
        self.pickup_commit_lost_min_radius_px = float(rospy.get_param("~pickup_commit_lost_min_radius_px", 38.0))
        self.pickup_commit_speed = float(rospy.get_param("~pickup_commit_speed_m_s", 0.08))
        self.pickup_commit_time_s = float(rospy.get_param("~pickup_commit_time_s", 0.8))
        self.marker_place_distance_m = float(rospy.get_param("~marker_place_distance_m", 0.45))
        self.marker_place_target_size_px = float(rospy.get_param("~marker_place_target_size_px", 180.0))
        self.marker_size_deadband_px = float(rospy.get_param("~marker_size_deadband_px", 10.0))
        self.marker_approach_base_speed = float(rospy.get_param("~marker_approach_base_speed_m_s", 0.06))
        self.marker_kp_size = float(rospy.get_param("~marker_kp_size", 0.002))
        self.search_timeout_s = float(rospy.get_param("~search_timeout_s", 25.0))
        self.approach_timeout_s = float(rospy.get_param("~approach_timeout_s", 12.0))
        self.place_search_timeout_s = float(rospy.get_param("~place_search_timeout_s", 60.0))

        self.wall_safety_enabled = bool(rospy.get_param("~wall_safety_enabled", True))
        self.front_stop_distance_m = float(rospy.get_param("~front_stop_distance_m", 0.18))
        self.front_slow_distance_m = float(rospy.get_param("~front_slow_distance_m", 0.35))
        self.scan_front_angle = math.radians(float(rospy.get_param("~scan_front_angle_deg", 38.0)))
        self.sensor_stale_s = float(rospy.get_param("~sensor_stale_s", 0.6))
        self.depth_safety_enabled = bool(rospy.get_param("~depth_safety_enabled", True))
        self.depth_safety_percentile = float(rospy.get_param("~depth_safety_percentile", 20.0))
        self.depth_safety_min_valid_fraction = float(rospy.get_param("~depth_safety_min_valid_fraction", 0.05))
        self.depth_roi_x_min = float(rospy.get_param("~depth_roi_x_min", 0.35))
        self.depth_roi_x_max = float(rospy.get_param("~depth_roi_x_max", 0.65))
        self.depth_roi_y_min = float(rospy.get_param("~depth_roi_y_min", 0.35))
        self.depth_roi_y_max = float(rospy.get_param("~depth_roi_y_max", 0.78))

        self.require_holding_feedback = bool(rospy.get_param("~require_holding_feedback", False))
        self.grasp_close_wait_s = float(rospy.get_param("~grasp_close_wait_s", 0.7))
        self.place_open_wait_s = float(rospy.get_param("~place_open_wait_s", 0.6))
        self.retreat_after_pick_s = float(rospy.get_param("~retreat_after_pick_s", 1.0))
        self.retreat_after_place_s = float(rospy.get_param("~retreat_after_place_s", 1.0))
        self.retreat_speed = float(rospy.get_param("~retreat_speed_m_s", 0.07))
        self.mission_timeout_s = float(rospy.get_param("~mission_timeout_s", 900.0))
        self.hold_terminal_state = bool(rospy.get_param("~hold_terminal_state", True))

        self.latest_pucks = []
        self.latest_puck_stamp = rospy.Time(0)
        self.latest_markers = {}
        self.latest_marker_stamp = rospy.Time(0)
        self.holding_object = False
        self.holding_stamp = rospy.Time(0)
        self.front_scan_min = None
        self.front_scan_stamp = rospy.Time(0)
        self.front_depth_m = None
        self.front_depth_stamp = rospy.Time(0)
        self.fx = self.fy = self.cx = self.cy = None
        self.image_width = None
        self.image_height = None

        self.delivered = set()
        self.active_color = ""
        self.started_at = rospy.Time.now()
        self.warned_no_safety_sensor = False

        self.aruco_dict = None
        self.aruco_params = None
        self.aruco_detector = None
        self._init_aruco()

        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.pub_state = rospy.Publisher("/mission/state", MissionState, queue_size=10, latch=True)

        rospy.Subscriber("/puck/detections", PuckDetectionArray, self._pucks_cb, queue_size=10)
        image_msg_type = CompressedImage if self.image_is_compressed else Image
        self.sub_image = [rospy.Subscriber(self.image_topic, image_msg_type, self._image_cb, queue_size=1)]
        if self.raw_image_fallback_topic:
            self.sub_image.append(rospy.Subscriber(self.raw_image_fallback_topic, Image, self._image_cb, queue_size=1))
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._depth_cb, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber(self.gripper_holding_topic, Bool, self._holding_cb, queue_size=10)

        rospy.wait_for_service(self.gripper_service)
        self.gripper_srv = rospy.ServiceProxy(self.gripper_service, SetGripper)

        rospy.on_shutdown(self._stop)
        rospy.loginfo(
            "challenge_manager ready: reactive no-AMCL sorter (image_topic=%s compressed=%s raw_fallback=%s)",
            self.image_topic,
            self.image_is_compressed,
            self.raw_image_fallback_topic,
        )

    @staticmethod
    def _normalize_color(color):
        return str(color).strip().lower()

    @classmethod
    def _normalize_color_list(cls, values):
        if isinstance(values, str):
            values = [v.strip() for v in values.split(",")]
        out = []
        for value in values or []:
            color = cls._normalize_color(value)
            if color and color not in out:
                out.append(color)
        return out

    @classmethod
    def _load_marker_map(cls, raw):
        out = {}
        for key, value in (raw or {}).items():
            try:
                marker_id = int(key)
            except Exception:
                continue
            color = cls._normalize_color(value)
            if color:
                out[marker_id] = color
        return out

    def _init_aruco(self):
        if not hasattr(cv2, "aruco"):
            rospy.logerr("OpenCV aruco module is unavailable; marker placement cannot run")
            return

        dict_id = getattr(cv2.aruco, self.aruco_dictionary_name, None)
        if dict_id is None:
            rospy.logwarn("unknown aruco dictionary %s; using DICT_4X4_50", self.aruco_dictionary_name)
            dict_id = cv2.aruco.DICT_4X4_50

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def _pucks_cb(self, msg):
        with self.lock:
            self.latest_pucks = list(msg.detections)
            self.latest_puck_stamp = rospy.Time.now()

    def _camera_info_cb(self, msg):
        if not self.use_camera_info_intrinsics:
            return
        with self.lock:
            self.fx = float(msg.K[0])
            self.fy = float(msg.K[4])
            self.cx = float(msg.K[2])
            self.cy = float(msg.K[5])

    def _holding_cb(self, msg):
        with self.lock:
            self.holding_object = bool(msg.data)
            self.holding_stamp = rospy.Time.now()

    def _scan_cb(self, msg):
        values = []
        angle = msg.angle_min
        for r in msg.ranges:
            if abs(angle) <= self.scan_front_angle and np.isfinite(r):
                if r >= msg.range_min and (msg.range_max <= 0.0 or r <= msg.range_max):
                    values.append(float(r))
            angle += msg.angle_increment
        with self.lock:
            self.front_scan_min = min(values) if values else None
            self.front_scan_stamp = rospy.Time.now()

    def _depth_cb(self, msg):
        if not self.depth_safety_enabled:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return
        if depth.dtype != np.float32:
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)
        h, w = depth.shape[:2]
        x0 = int(clamp(self.depth_roi_x_min, 0.0, 1.0) * w)
        x1 = int(clamp(self.depth_roi_x_max, 0.0, 1.0) * w)
        y0 = int(clamp(self.depth_roi_y_min, 0.0, 1.0) * h)
        y1 = int(clamp(self.depth_roi_y_max, 0.0, 1.0) * h)
        roi = depth[max(0, y0):max(0, y1), max(0, x0):max(0, x1)]
        if roi.size == 0:
            return
        valid = roi[np.isfinite(roi)]
        valid = valid[valid > 0.05]
        min_valid = max(1, int(roi.size * self.depth_safety_min_valid_fraction))
        depth_m = None
        if valid.size >= min_valid:
            depth_m = float(np.percentile(valid, self.depth_safety_percentile))
        with self.lock:
            self.front_depth_m = depth_m
            self.front_depth_stamp = rospy.Time.now()

    def _bgr_from_msg(self, msg):
        if isinstance(msg, CompressedImage):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("compressed image decode returned None")
            return frame
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _camera_matrix(self, width, height):
        with self.lock:
            fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        if self.use_camera_info_intrinsics and fx and fy:
            return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
        hfov = math.radians(self.hfov_deg)
        fx = (width / 2.0) / max(1e-6, math.tan(hfov / 2.0))
        return np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]], dtype=np.float32)

    def _estimate_marker_tvec(self, marker_corners, camera_matrix, dist_coeffs):
        corners = marker_corners.astype(np.float32).reshape(1, 4, 2)
        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            _rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size_m, camera_matrix, dist_coeffs)
            if tvecs is None or len(tvecs) == 0:
                return None
            return tvecs[0][0]

        half = self.marker_size_m / 2.0
        obj = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float32,
        )
        img = marker_corners.reshape(4, 2).astype(np.float32)
        ok, _rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        return tvec.reshape(3)

    def _image_cb(self, msg):
        if self.aruco_dict is None:
            return
        try:
            frame = self._bgr_from_msg(msg)
        except Exception:
            return
        h, w = frame.shape[:2]
        with self.lock:
            self.image_width = w
            self.image_height = h

        if self.aruco_detector is not None:
            corners, ids, _rejected = self.aruco_detector.detectMarkers(frame)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            with self.lock:
                self.latest_markers = {}
                self.latest_marker_stamp = rospy.Time.now()
            return

        camera_matrix = self._camera_matrix(w, h)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        now = rospy.Time.now()
        markers = {}

        for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
            marker_id = int(marker_id)
            color = self.marker_id_to_color.get(marker_id, "")
            if not color:
                continue
            pts = marker_corners.reshape(-1, 2).astype(np.float32)
            sides = [np.linalg.norm(pts[j] - pts[(j + 1) % 4]) for j in range(4)]
            side_px = float(max(sides))
            if cv2.arcLength(pts.reshape(-1, 1, 2), True) < self.min_aruco_perimeter_px:
                continue
            tvec = self._estimate_marker_tvec(marker_corners, camera_matrix, dist_coeffs)
            if tvec is None:
                continue
            x = float(tvec[0])
            z = float(tvec[2])
            if z <= 0.0:
                continue
            obs = MarkerObservation(marker_id=marker_id, color=color, x=x, z=z, stamp=now, distance_m=float(np.linalg.norm(tvec)), size_px=side_px)
            prev = markers.get(color)
            if prev is None or obs.z < prev.z:
                markers[color] = obs

        with self.lock:
            self.latest_markers = markers
            self.latest_marker_stamp = now

    def _publish_state(self, state, note):
        msg = MissionState()
        msg.header.stamp = rospy.Time.now()
        msg.state = state
        msg.active_track_id = 0
        msg.remaining_estimate = len([c for c in self.expected_colors if c not in self.delivered])
        msg.finished = state == "FINISHED"
        msg.note = note
        self.pub_state.publish(msg)

    def _terminal(self, state, note):
        self._stop()
        self._publish_state(state, note)
        rospy.loginfo("challenge_manager terminal state %s: %s", state, note)
        if self.hold_terminal_state:
            rate = rospy.Rate(1.0)
            while not rospy.is_shutdown():
                self._publish_state(state, note)
                rate.sleep()

    def _stop(self):
        self.pub_cmd.publish(Twist())

    def _front_distance(self):
        now = rospy.Time.now()
        distances = []
        with self.lock:
            if self.front_scan_min is not None and (now - self.front_scan_stamp).to_sec() <= self.sensor_stale_s:
                distances.append(self.front_scan_min)
            if self.front_depth_m is not None and (now - self.front_depth_stamp).to_sec() <= self.sensor_stale_s:
                distances.append(self.front_depth_m)
        if not distances:
            if self.wall_safety_enabled and not self.warned_no_safety_sensor:
                rospy.logwarn("no fresh /scan or depth safety data; forward wall safety is limited")
                self.warned_no_safety_sensor = True
            return None
        return min(distances)

    def _forward_scale(self):
        if not self.wall_safety_enabled:
            return 1.0
        dist = self._front_distance()
        if dist is None:
            return 1.0
        if dist <= self.front_stop_distance_m:
            return 0.0
        if dist >= self.front_slow_distance_m:
            return 1.0
        span = max(1e-6, self.front_slow_distance_m - self.front_stop_distance_m)
        return clamp((dist - self.front_stop_distance_m) / span, 0.0, 1.0)

    def _cmd(self, linear_x, angular_z):
        cmd = Twist()
        scale = self._forward_scale() if linear_x > 0.0 else 1.0
        cmd.linear.x = clamp(linear_x * scale, -self.max_linear_speed, self.max_linear_speed)
        cmd.angular.z = clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)
        self.pub_cmd.publish(cmd)

    def _aim_lateral_m(self, depth_m):
        if abs(self.gripper_aim_offset_px) < 1e-6:
            return 0.0
        with self.lock:
            fx = self.fx
            image_width = self.image_width
        if not fx or fx <= 0.0:
            image_width = image_width or 640
            hfov = math.radians(self.hfov_deg)
            fx = (float(image_width) / 2.0) / max(1e-6, math.tan(hfov / 2.0))
        return self.gripper_aim_offset_px * depth_m / fx

    def _lateral_error(self, observed_x_m, depth_m):
        return observed_x_m - self._aim_lateral_m(depth_m)

    def _retreat(self, duration_s):
        rate = rospy.Rate(self.control_rate_hz)
        end = rospy.Time.now() + rospy.Duration(max(0.0, duration_s))
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self._cmd(-abs(self.retreat_speed), 0.0)
            rate.sleep()
        self._stop()

    def _commit_pickup(self):
        rate = rospy.Rate(self.control_rate_hz)
        end = rospy.Time.now() + rospy.Duration(max(0.0, self.pickup_commit_time_s))
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self._cmd(abs(self.pickup_commit_speed), 0.0)
            rate.sleep()
        self._stop()
        return True

    def _select_puck(self, color):
        now = rospy.Time.now()
        with self.lock:
            if (now - self.latest_puck_stamp).to_sec() > self.puck_stale_s:
                return None
            matches = [
                det for det in self.latest_pucks
                if self._normalize_color(det.color) == color
                and det.confidence >= self.puck_conf_min
                and det.position_camera.z > 0.0
            ]
        if not matches:
            return None
        return min(matches, key=lambda det: det.position_camera.z)

    def _select_any_puck_color(self, colors):
        now = rospy.Time.now()
        wanted = set([self._normalize_color(color) for color in colors])
        with self.lock:
            if (now - self.latest_puck_stamp).to_sec() > self.puck_stale_s:
                return None
            matches = [
                det for det in self.latest_pucks
                if self._normalize_color(det.color) in wanted
                and det.confidence >= self.puck_conf_min
                and det.position_camera.z > 0.0
            ]
        if not matches:
            return None
        best = min(matches, key=lambda det: det.position_camera.z)
        return self._normalize_color(best.color)

    def _select_marker(self, color):
        now = rospy.Time.now()
        with self.lock:
            marker = self.latest_markers.get(color)
        if marker is None:
            return None
        if (now - marker.stamp).to_sec() > self.marker_stale_s:
            return None
        return marker

    def _search_for(self, target_type, color, timeout_s):
        rate = rospy.Rate(self.control_rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.started_at).to_sec() > self.mission_timeout_s:
                return None
            if timeout_s > 0.0 and (rospy.Time.now() - start).to_sec() > timeout_s:
                return None
            if target_type == "puck":
                obj = self._select_puck(color)
            else:
                obj = self._select_marker(color)
            if obj is not None:
                self._stop()
                return obj
            self._cmd(0.0, self.search_angular_speed)
            rate.sleep()
        self._stop()
        return None

    def _search_for_any_puck(self, colors, timeout_s):
        rate = rospy.Rate(self.control_rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.started_at).to_sec() > self.mission_timeout_s:
                return None
            if timeout_s > 0.0 and (rospy.Time.now() - start).to_sec() > timeout_s:
                return None
            color = self._select_any_puck_color(colors)
            if color is not None:
                self._stop()
                return color
            self._cmd(0.0, self.search_angular_speed)
            rate.sleep()
        self._stop()
        return None

    def _approach_puck(self, color):
        rate = rospy.Rate(self.control_rate_hz)
        start = rospy.Time.now()
        stable_since = None
        last_radius_px = 0.0
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() <= self.approach_timeout_s:
            det = self._select_puck(color)
            if det is None:
                if self.pickup_commit_enabled and last_radius_px >= self.pickup_commit_lost_min_radius_px:
                    return self._commit_pickup()
                self._stop()
                return False

            radius_px = math.sqrt(max(0.0, float(det.contour_area_px)) / math.pi)
            if radius_px > 0.0:
                last_radius_px = radius_px
            if self.pickup_commit_enabled and radius_px >= self.pickup_commit_radius_px:
                return self._commit_pickup()

            depth = det.position_camera.z
            lateral = self._lateral_error(det.position_camera.x, depth)
            depth_err = depth - self.pickup_target_depth_m
            aligned = abs(lateral) <= self.lateral_deadband_m and abs(depth_err) <= self.depth_deadband_m
            if aligned:
                if stable_since is None:
                    stable_since = rospy.Time.now()
                if (rospy.Time.now() - stable_since).to_sec() >= self.align_hold_s:
                    self._stop()
                    return True
            else:
                stable_since = None

            angular = -self.kp_yaw * lateral
            if self.pickup_commit_enabled and radius_px > 0.0:
                radius_err = self.pickup_commit_radius_px - radius_px
                linear = self.approach_base_speed + 0.003 * radius_err
            else:
                linear = self.kp_depth * depth_err
            if abs(lateral) >= 0.12:
                linear = 0.0
            if linear < 0.0:
                linear = 0.0
            self._cmd(linear, angular)
            rate.sleep()
        self._stop()
        return False

    def _approach_marker(self, color):
        rate = rospy.Rate(self.control_rate_hz)
        start = rospy.Time.now()
        stable_since = None
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() <= self.approach_timeout_s:
            marker = self._select_marker(color)
            if marker is None:
                self._stop()
                return False

            depth = marker.z
            lateral = self._lateral_error(marker.x, depth)
            depth_err = depth - self.marker_place_distance_m
            size_err = self.marker_place_target_size_px - marker.size_px
            if self.marker_place_target_size_px > 0.0:
                aligned = abs(lateral) <= self.lateral_deadband_m and abs(size_err) <= self.marker_size_deadband_px
            else:
                aligned = abs(lateral) <= self.lateral_deadband_m and abs(depth_err) <= self.depth_deadband_m
            if aligned:
                if stable_since is None:
                    stable_since = rospy.Time.now()
                if (rospy.Time.now() - stable_since).to_sec() >= self.align_hold_s:
                    self._stop()
                    return True
            else:
                stable_since = None

            angular = -self.kp_yaw * lateral
            if self.marker_place_target_size_px > 0.0:
                linear = self.marker_approach_base_speed + self.marker_kp_size * size_err
            else:
                linear = self.kp_depth * depth_err
            if abs(lateral) >= 0.15:
                linear = 0.0
            if linear < 0.0:
                linear = 0.0
            self._cmd(linear, angular)
            rate.sleep()
        self._stop()
        return False

    def _set_gripper(self, command):
        try:
            rsp = self.gripper_srv(command=command, angle_deg=0.0)
        except Exception as exc:
            return False, str(exc)
        return bool(rsp.success), rsp.message

    def _holding_recent(self):
        with self.lock:
            holding = self.holding_object
            stamp = self.holding_stamp
        return holding and stamp != rospy.Time(0) and (rospy.Time.now() - stamp).to_sec() <= 2.0

    def _wait_for_holding(self, timeout_s):
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self._holding_recent():
                return True
            rate.sleep()
        return False

    def _pick_color(self, color):
        self.active_color = color
        self._publish_state("SEARCH_PUCK", f"searching for {color} puck")
        if self._search_for("puck", color, self.search_timeout_s) is None:
            return False, f"{color} puck not visible"

        self._publish_state("APPROACH_PUCK", f"approaching {color} puck")
        if not self._approach_puck(color):
            return False, f"could not align to {color} puck"

        ok, msg = self._set_gripper("close")
        if not ok:
            return False, f"gripper close failed: {msg}"
        rospy.sleep(self.grasp_close_wait_s)

        if self.require_holding_feedback and not self._wait_for_holding(1.0):
            self._set_gripper("open")
            return False, f"{color} pickup not confirmed by gripper feedback"

        self._publish_state("RETREAT_PICK", f"{color} puck picked")
        self._retreat(self.retreat_after_pick_s)
        return True, "picked"

    def _place_color(self, color):
        self._publish_state("SEARCH_MARKER", f"searching for ArUco home marker for {color}")
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.started_at).to_sec() > self.mission_timeout_s:
                return False, "mission timeout while searching for marker"
            marker = self._search_for("marker", color, self.place_search_timeout_s)
            if marker is None:
                self._publish_state("SEARCH_MARKER", f"still searching for ArUco home marker for {color}")
                continue

            self._publish_state("APPROACH_MARKER", f"approaching {color} home marker ID {marker.marker_id}")
            if self._approach_marker(color):
                break

        ok, msg = self._set_gripper("open")
        if not ok:
            return False, f"gripper open failed: {msg}"
        rospy.sleep(self.place_open_wait_s)
        self._publish_state("RETREAT_PLACE", f"placed {color} puck")
        self._retreat(self.retreat_after_place_s)
        self.delivered.add(color)
        return True, "placed"

    def run(self):
        ok, msg = self._set_gripper("open")
        if not ok:
            self._terminal("ERROR", f"initial gripper open failed: {msg}")
            return

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.started_at).to_sec() > self.mission_timeout_s:
                self._terminal("TIMEOUT", "mission timeout")
                return

            remaining = [color for color in self.expected_colors if color not in self.delivered]
            if not remaining:
                self._terminal("FINISHED", "all pucks placed")
                return

            self._publish_state("SEARCH_PUCK", "searching for any remaining puck")
            color = self._search_for_any_puck(remaining, self.search_timeout_s)
            if color is None:
                self._publish_state("SEARCH_PUCK", "no expected puck visible; rotating to search")
                self._cmd(0.0, self.search_angular_speed)
                rate.sleep()
                continue

            picked, pick_msg = self._pick_color(color)
            if not picked:
                self._publish_state("SEARCH_PUCK", pick_msg)
                continue
            placed, place_msg = self._place_color(color)
            if not placed:
                self._terminal("ERROR", place_msg)
                return


def main():
    rospy.init_node("challenge_manager")
    manager = ChallengeManager()
    manager.run()


if __name__ == "__main__":
    main()
