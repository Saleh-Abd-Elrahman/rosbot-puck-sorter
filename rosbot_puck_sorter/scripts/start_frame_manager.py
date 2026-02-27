#!/usr/bin/env python3
import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped
from std_msgs.msg import Bool


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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


class StartFrameManager:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.start_frame = rospy.get_param("~start_frame", "start")
        self.amcl_topic = rospy.get_param("~amcl_topic", "/amcl_pose")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 15.0))

        self.initialized = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 0.0
        self.start_yaw = 0.0

        self.latest_pose = None

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.pub_init = rospy.Publisher("/start_frame/initialized", Bool, queue_size=1, latch=True)
        self.pub_origin = rospy.Publisher("/start_frame/origin_map", PoseStamped, queue_size=1, latch=True)
        self.pub_robot_start = rospy.Publisher("/robot_pose_start", PoseStamped, queue_size=10)

        self.pub_init.publish(Bool(data=False))

        rospy.Subscriber(self.amcl_topic, PoseWithCovarianceStamped, self._amcl_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-3, self.publish_rate_hz)), self._timer_cb)

        rospy.loginfo("start_frame_manager ready: waiting for first %s", self.amcl_topic)

    def _broadcast_start_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.start_frame

        c = math.cos(self.start_yaw)
        s = math.sin(self.start_yaw)

        # map -> start transform (inverse of start pose in map frame)
        tf_msg.transform.translation.x = -(c * self.start_x + s * self.start_y)
        tf_msg.transform.translation.y = (s * self.start_x - c * self.start_y)
        tf_msg.transform.translation.z = -self.start_z

        q = quat_from_yaw(-self.start_yaw)
        tf_msg.transform.rotation = q

        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_origin(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = self.start_x
        msg.pose.position.y = self.start_y
        msg.pose.position.z = self.start_z
        msg.pose.orientation = quat_from_yaw(self.start_yaw)
        self.pub_origin.publish(msg)

    def _amcl_cb(self, msg):
        self.latest_pose = msg.pose.pose

        if self.initialized:
            return

        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        self.start_z = msg.pose.pose.position.z
        self.start_yaw = yaw_from_quat(msg.pose.pose.orientation)

        self._broadcast_start_tf()
        self._publish_origin()
        self.pub_init.publish(Bool(data=True))

        self.initialized = True
        rospy.loginfo(
            "start frame initialized at map pose x=%.3f y=%.3f yaw=%.3f rad",
            self.start_x,
            self.start_y,
            self.start_yaw,
        )

    def _timer_cb(self, _event):
        if not self.initialized or self.latest_pose is None:
            return

        current = self.latest_pose
        dx = current.position.x - self.start_x
        dy = current.position.y - self.start_y

        c = math.cos(self.start_yaw)
        s = math.sin(self.start_yaw)

        x_rel = c * dx + s * dy
        y_rel = -s * dx + c * dy
        yaw_rel = wrap_pi(yaw_from_quat(current.orientation) - self.start_yaw)

        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.start_frame
        out.pose.position.x = x_rel
        out.pose.position.y = y_rel
        out.pose.position.z = current.position.z - self.start_z
        out.pose.orientation = quat_from_yaw(yaw_rel)
        self.pub_robot_start.publish(out)


def main():
    rospy.init_node("start_frame_manager")
    StartFrameManager()
    rospy.spin()


if __name__ == "__main__":
    main()
