#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SimplePoseNavigator:
    def __init__(self):
        self.lock = threading.Lock()

        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
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

        self.have_pose = False
        self.pose_stamp = rospy.Time(0)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self._pose_cb, queue_size=10)

    def _pose_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = yaw
            self.pose_stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
            self.have_pose = True

    def wait_for_pose(self, timeout_s=None):
        timeout_s = self.pose_wait_timeout_s if timeout_s is None else float(timeout_s)
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.have_pose:
                    return True
            rate.sleep()

        return False

    def _current_pose(self):
        with self.lock:
            return self.have_pose, self.robot_x, self.robot_y, self.robot_yaw

    def stop(self):
        self.pub_cmd.publish(Twist())

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def goto(self, x, y, yaw, timeout_s=45.0):
        pose_wait_timeout = min(self.pose_wait_timeout_s, float(timeout_s))
        if not self.wait_for_pose(timeout_s=pose_wait_timeout):
            rospy.logwarn("simple_pose_navigator: no pose on %s", self.pose_topic)
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
