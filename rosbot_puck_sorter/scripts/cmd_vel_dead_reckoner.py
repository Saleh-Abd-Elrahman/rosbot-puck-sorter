#!/usr/bin/env python3
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CmdVelDeadReckoner:
    def __init__(self):
        self.lock = threading.Lock()

        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        self.command_timeout_s = float(rospy.get_param("~command_timeout_s", 0.35))
        self.linear_scale = float(rospy.get_param("~linear_scale", 1.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.0))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_cmd = Twist()
        self.last_cmd_stamp = rospy.Time(0)
        self.last_update = rospy.Time.now()

        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-3, self.publish_rate_hz)), self._timer_cb)

        rospy.logwarn(
            "cmd_vel_dead_reckoner publishing approximate %s and TF %s->%s from %s; use real odom when available",
            self.odom_topic,
            self.odom_frame,
            self.base_frame,
            self.cmd_topic,
        )

    def _cmd_cb(self, msg):
        with self.lock:
            self.last_cmd = msg
            self.last_cmd_stamp = rospy.Time.now()

    def _timer_cb(self, _event):
        now = rospy.Time.now()
        dt = max(0.0, (now - self.last_update).to_sec())
        self.last_update = now

        with self.lock:
            cmd = self.last_cmd
            fresh = self.last_cmd_stamp != rospy.Time(0) and (now - self.last_cmd_stamp).to_sec() <= self.command_timeout_s

        vx = self.linear_scale * cmd.linear.x if fresh else 0.0
        wz = self.angular_scale * cmd.angular.z if fresh else 0.0

        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw = wrap_pi(self.yaw + wz * dt)

        q = quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rospy.init_node("cmd_vel_dead_reckoner")
    CmdVelDeadReckoner()
    rospy.spin()


if __name__ == "__main__":
    main()
