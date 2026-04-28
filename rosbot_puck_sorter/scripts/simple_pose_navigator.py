#!/usr/bin/env python3
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry


def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SimplePoseNavigator:
    def __init__(self):
        self.lock = threading.Lock()

        self.pose_source = str(rospy.get_param("~nav_pose_source", rospy.get_param("~pose_source", "auto"))).strip().lower()
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.control_rate_hz = float(rospy.get_param("~nav_control_rate_hz", 20.0))
        self.max_linear_speed = float(rospy.get_param("~nav_max_linear_speed_m_s", 0.18))
        self.max_angular_speed = float(rospy.get_param("~nav_max_angular_speed_rad_s", 0.9))
        self.linear_kp = float(rospy.get_param("~nav_linear_kp", 0.7))
        self.angular_kp = float(rospy.get_param("~nav_angular_kp", 1.8))
        self.drive_angular_kp = float(rospy.get_param("~nav_drive_angular_kp", 1.2))
        self.rotate_in_place_threshold = float(rospy.get_param("~nav_rotate_in_place_threshold_rad", 0.45))
        self.goal_xy_tolerance = float(rospy.get_param("~goal_xy_tolerance_m", rospy.get_param("~waypoint_reached_tol_m", 0.12)))
        self.goal_yaw_tolerance = float(rospy.get_param("~goal_yaw_tolerance_rad", rospy.get_param("~yaw_tolerance_rad", 0.18)))
        self.slowing_distance = float(rospy.get_param("~nav_slowing_distance_m", 0.50))
        self.success_hold_s = float(rospy.get_param("~nav_success_hold_s", 0.20))
        self.pose_wait_timeout_s = float(rospy.get_param("~nav_pose_wait_timeout_s", 2.0))
        self.pose_stale_timeout_s = float(rospy.get_param("~nav_pose_stale_timeout_s", 3.0))
        self.allow_open_loop_fallback = bool(rospy.get_param("~nav_allow_open_loop_fallback", False))
        self.open_loop_linear_speed = float(rospy.get_param("~nav_open_loop_linear_speed_m_s", min(self.max_linear_speed, 0.10)))
        self.open_loop_angular_speed = float(rospy.get_param("~nav_open_loop_angular_speed_rad_s", min(self.max_angular_speed, 0.45)))
        self.open_loop_distance_scale = float(rospy.get_param("~nav_open_loop_distance_scale", 1.0))
        self.open_loop_turn_scale = float(rospy.get_param("~nav_open_loop_turn_scale", 1.0))

        self.have_pose = False
        self.pose_stamp = rospy.Time(0)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.amcl_pose = None
        self.odom_pose = None
        self.open_loop_x = 0.0
        self.open_loop_y = 0.0
        self.open_loop_yaw = 0.0

        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.tf_buffer = None
        self.tf_listener = None

        if self.pose_source in ("auto", "amcl", "pose", "pose_topic"):
            rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self._pose_cb, queue_size=10)
        if self.pose_source in ("auto", "odom"):
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        if self.pose_source in ("auto", "tf", "odom", "amcl", "pose", "pose_topic"):
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _pose_cb(self, msg):
        self._store_pose("amcl", msg.header.stamp, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quat(msg.pose.pose.orientation))

    def _odom_cb(self, msg):
        self._store_pose("odom", msg.header.stamp, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quat(msg.pose.pose.orientation))

    def _store_pose(self, source, stamp, x, y, yaw):
        stamp = stamp if stamp != rospy.Time(0) else rospy.Time.now()
        with self.lock:
            pose = (stamp, float(x), float(y), float(yaw))
            if source == "amcl":
                self.amcl_pose = pose
            elif source == "odom":
                self.odom_pose = pose
            self.robot_x = float(x)
            self.robot_y = float(y)
            self.robot_yaw = yaw
            self.pose_stamp = stamp
            self.have_pose = True

    def _pose_is_fresh(self, pose, now):
        if pose is None:
            return False
        if self.pose_stale_timeout_s <= 0.0:
            return True
        return (now - pose[0]).to_sec() <= self.pose_stale_timeout_s

    def _stored_pose(self):
        now = rospy.Time.now()
        with self.lock:
            if self.pose_source in ("amcl", "pose", "pose_topic"):
                order = [self.amcl_pose]
            elif self.pose_source == "odom":
                order = [self.odom_pose]
            elif self.pose_source == "auto":
                order = [self.amcl_pose, self.odom_pose]
            else:
                order = []

            for pose in order:
                if self._pose_is_fresh(pose, now):
                    return pose

        return None

    def _tf_pose(self):
        if self.tf_buffer is None:
            return None

        frames = []
        if self.pose_source == "tf":
            frames.append((self.map_frame, self.base_frame))
        else:
            frames.append((self.map_frame, self.base_frame))
            if self.odom_frame != self.map_frame:
                frames.append((self.odom_frame, self.base_frame))

        for target_frame, source_frame in frames:
            try:
                t = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), timeout=rospy.Duration(0.03))
            except Exception:
                continue
            q = t.transform.rotation
            return (
                t.header.stamp if t.header.stamp != rospy.Time(0) else rospy.Time.now(),
                t.transform.translation.x,
                t.transform.translation.y,
                yaw_from_quat(q),
            )

        return None

    def wait_for_pose(self, timeout_s=None):
        timeout_s = self.pose_wait_timeout_s if timeout_s is None else float(timeout_s)
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self._current_pose()[0]:
                return True
            rate.sleep()

        return False

    def _current_pose(self):
        pose = self._stored_pose()
        if pose is None and self.pose_source in ("auto", "tf", "odom", "amcl", "pose", "pose_topic"):
            pose = self._tf_pose()
        if pose is None:
            return False, 0.0, 0.0, 0.0
        return True, pose[1], pose[2], pose[3]

    def current_pose(self):
        return self._current_pose()

    def stop(self):
        self.pub_cmd.publish(Twist())

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def _publish_open_loop(self, linear_x, angular_z, duration_s, deadline):
        duration_s = max(0.0, float(duration_s))
        if duration_s <= 1e-6:
            return True

        rate = rospy.Rate(self.control_rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration_s:
            if deadline is not None and rospy.Time.now() >= deadline:
                self.stop()
                return False
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            self.pub_cmd.publish(cmd)
            rate.sleep()
        self.stop()
        return not rospy.is_shutdown()

    def _rotate_open_loop(self, angle_rad, deadline):
        if abs(angle_rad) <= self.goal_yaw_tolerance:
            return True
        speed = max(0.05, abs(self.open_loop_angular_speed))
        direction = 1.0 if angle_rad >= 0.0 else -1.0
        duration = (abs(angle_rad) / speed) * max(0.05, self.open_loop_turn_scale)
        return self._publish_open_loop(0.0, direction * speed, duration, deadline)

    def _drive_open_loop(self, distance_m, deadline):
        if abs(distance_m) <= self.goal_xy_tolerance:
            return True
        speed = max(0.02, abs(self.open_loop_linear_speed))
        direction = 1.0 if distance_m >= 0.0 else -1.0
        duration = (abs(distance_m) / speed) * max(0.05, self.open_loop_distance_scale)
        return self._publish_open_loop(direction * speed, 0.0, duration, deadline)

    def _goto_open_loop(self, x, y, yaw, timeout_s):
        deadline = rospy.Time.now() + rospy.Duration(float(timeout_s))
        with self.lock:
            start_x = self.open_loop_x
            start_y = self.open_loop_y
            start_yaw = self.open_loop_yaw

        dx = float(x) - start_x
        dy = float(y) - start_y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if distance > self.goal_xy_tolerance else start_yaw

        if not self._rotate_open_loop(angle_wrap(bearing - start_yaw), deadline):
            return False
        if not self._drive_open_loop(distance, deadline):
            return False
        if not self._rotate_open_loop(angle_wrap(float(yaw) - bearing), deadline):
            return False

        with self.lock:
            self.open_loop_x = float(x)
            self.open_loop_y = float(y)
            self.open_loop_yaw = float(yaw)
        return True

    def goto(self, x, y, yaw, timeout_s=45.0):
        if self.pose_source in ("open_loop", "cmd_vel", "cmd_vel_only"):
            return self._goto_open_loop(x, y, yaw, timeout_s)

        pose_wait_timeout = min(self.pose_wait_timeout_s, float(timeout_s))
        if not self.wait_for_pose(timeout_s=pose_wait_timeout):
            if self.allow_open_loop_fallback:
                rospy.logwarn(
                    "simple_pose_navigator: no pose from %s/%s/TF; using open-loop cmd_vel fallback",
                    self.pose_topic,
                    self.odom_topic,
                )
                return self._goto_open_loop(x, y, yaw, timeout_s)
            rospy.logwarn("simple_pose_navigator: no pose from %s/%s/TF", self.pose_topic, self.odom_topic)
            self.stop()
            return False

        rate = rospy.Rate(self.control_rate_hz)
        deadline = rospy.Time.now() + rospy.Duration(float(timeout_s))
        stable_since = None

        while not rospy.is_shutdown():
            if rospy.Time.now() >= deadline:
                self.stop()
                return False

            have_pose, robot_x, robot_y, robot_yaw = self._current_pose()
            if not have_pose:
                self.stop()
                rate.sleep()
                continue

            dx = float(x) - robot_x
            dy = float(y) - robot_y
            dist = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx) if dist > 1e-6 else float(yaw)
            heading_err = angle_wrap(bearing - robot_yaw)
            final_yaw_err = angle_wrap(float(yaw) - robot_yaw)

            cmd = Twist()

            if dist <= self.goal_xy_tolerance:
                if abs(final_yaw_err) <= self.goal_yaw_tolerance:
                    if stable_since is None:
                        stable_since = rospy.Time.now()
                    if (rospy.Time.now() - stable_since).to_sec() >= self.success_hold_s:
                        self.stop()
                        return True
                else:
                    stable_since = None
                    cmd.angular.z = self._clamp(self.angular_kp * final_yaw_err, self.max_angular_speed)
            else:
                stable_since = None
                if abs(heading_err) >= self.rotate_in_place_threshold:
                    cmd.angular.z = self._clamp(self.angular_kp * heading_err, self.max_angular_speed)
                else:
                    linear = min(self.max_linear_speed, self.linear_kp * dist)
                    if self.slowing_distance > 1e-6:
                        linear *= max(0.25, min(1.0, dist / self.slowing_distance))
                    heading_scale = max(0.15, 1.0 - abs(heading_err) / max(self.rotate_in_place_threshold, 1e-6))
                    cmd.linear.x = linear * heading_scale
                    cmd.angular.z = self._clamp(self.drive_angular_kp * heading_err, self.max_angular_speed)

            self.pub_cmd.publish(cmd)
            rate.sleep()

        self.stop()
        return False
