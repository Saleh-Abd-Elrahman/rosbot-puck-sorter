#!/usr/bin/env python3
import math

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
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
        self.pose_source = str(rospy.get_param("~pose_source", "auto")).strip().lower()
        self.amcl_topic = rospy.get_param("~amcl_topic", "/amcl_pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 15.0))

        self.initialized = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 0.0
        self.start_yaw = 0.0

        self.latest_pose = None
        self.active_pose_source = ""

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = None
        self.tf_listener = None

        self.pub_init = rospy.Publisher("/start_frame/initialized", Bool, queue_size=1, latch=True)
        self.pub_origin = rospy.Publisher("/start_frame/origin_map", PoseStamped, queue_size=1, latch=True)
        self.pub_robot_start = rospy.Publisher("/robot_pose_start", PoseStamped, queue_size=10)

        self.pub_init.publish(Bool(data=False))

        if self.pose_source in ("auto", "amcl", "pose", "pose_topic"):
            rospy.Subscriber(self.amcl_topic, PoseWithCovarianceStamped, self._amcl_cb, queue_size=20)
        if self.pose_source in ("auto", "odom"):
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=20)
        if self.pose_source in ("auto", "tf", "odom", "amcl", "pose", "pose_topic"):
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-3, self.publish_rate_hz)), self._timer_cb)

        rospy.loginfo(
            "start_frame_manager ready: waiting for pose source=%s amcl=%s odom=%s",
            self.pose_source,
            self.amcl_topic,
            self.odom_topic,
        )

    def _broadcast_start_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.start_frame

        # TF stores the child frame pose in the parent frame. TF consumers apply
        # the inverse automatically when transforming map-frame points into start.
        tf_msg.transform.translation.x = self.start_x
        tf_msg.transform.translation.y = self.start_y
        tf_msg.transform.translation.z = self.start_z
        tf_msg.transform.rotation = quat_from_yaw(self.start_yaw)

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
        self._pose_cb(msg.pose.pose, "amcl")

    def _odom_cb(self, msg):
        self._pose_cb(msg.pose.pose, "odom")

    def _pose_cb(self, pose, source):
        if self.active_pose_source and source != self.active_pose_source:
            return

        self.latest_pose = pose
        if self.initialized:
            return

        self.start_x = pose.position.x
        self.start_y = pose.position.y
        self.start_z = pose.position.z
        self.start_yaw = yaw_from_quat(pose.orientation)

        self._broadcast_start_tf()
        self._publish_origin()
        self.pub_init.publish(Bool(data=True))

        self.initialized = True
        self.active_pose_source = source
        rospy.loginfo(
            "start frame initialized from %s at %s pose x=%.3f y=%.3f yaw=%.3f rad",
            source,
            self.map_frame,
            self.start_x,
            self.start_y,
            self.start_yaw,
        )

    def _update_from_tf(self):
        if self.tf_buffer is None:
            return False
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), timeout=rospy.Duration(0.02))
        except Exception:
            return False

        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        source = "odom" if self.pose_source == "odom" else "tf"
        self._pose_cb(pose, source)
        return True

    def _timer_cb(self, _event):
        if self.pose_source in ("auto", "tf", "odom", "amcl", "pose", "pose_topic"):
            self._update_from_tf()

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
