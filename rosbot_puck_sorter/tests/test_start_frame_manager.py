#!/usr/bin/env python3
import math

import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool

from common import load_script_module, safe_shutdown, wait_for, yaw_to_quat


def main():
    rospy.init_node("test_start_frame_manager")

    rospy.set_param("~map_frame", "map")
    rospy.set_param("~start_frame", "start")
    rospy.set_param("~amcl_topic", "/amcl_pose")
    rospy.set_param("~publish_rate_hz", 20.0)

    module = load_script_module("start_frame_manager.py", "start_frame_manager_test_mod")
    module.StartFrameManager()
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
    tf2_ros.TransformListener(tf_buffer)

    pub_amcl = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size=10)

    initialized = {"value": False}
    latest_robot = {"msg": None}

    def init_cb(msg):
        initialized["value"] = bool(msg.data)

    def robot_cb(msg):
        latest_robot["msg"] = msg

    rospy.Subscriber("/start_frame/initialized", Bool, init_cb, queue_size=1)
    rospy.Subscriber("/robot_pose_start", PoseStamped, robot_cb, queue_size=10)

    # First pose defines start frame.
    p0 = PoseWithCovarianceStamped()
    p0.header.frame_id = "map"
    p0.pose.pose.position.x = 1.0
    p0.pose.pose.position.y = 2.0
    p0.pose.pose.orientation = yaw_to_quat(0.0)

    # Second pose should become relative x=0.5, y=0.3.
    p1 = PoseWithCovarianceStamped()
    p1.header.frame_id = "map"
    p1.pose.pose.position.x = 1.5
    p1.pose.pose.position.y = 2.3
    p1.pose.pose.orientation = yaw_to_quat(0.0)

    rate = rospy.Rate(20)
    for _ in range(8):
        p0.header.stamp = rospy.Time.now()
        pub_amcl.publish(p0)
        rate.sleep()

    wait_for(lambda: initialized["value"], timeout_s=2.0, desc="start frame initialization")

    for _ in range(20):
        p1.header.stamp = rospy.Time.now()
        pub_amcl.publish(p1)
        rate.sleep()

    wait_for(lambda: latest_robot["msg"] is not None, timeout_s=2.0, desc="robot_pose_start")

    msg = latest_robot["msg"]
    dx = abs(msg.pose.position.x - 0.5)
    dy = abs(msg.pose.position.y - 0.3)
    if dx > 0.08 or dy > 0.08:
        raise RuntimeError(f"unexpected start-relative pose: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}")

    yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
    if abs(yaw) > 0.1:
        raise RuntimeError(f"unexpected relative yaw: {yaw:.3f}")

    pt_map = PointStamped()
    pt_map.header.stamp = rospy.Time(0)
    pt_map.header.frame_id = "map"
    pt_map.point.x = 1.5
    pt_map.point.y = 2.3
    pt_map.point.z = 0.0

    pt_start = tf_buffer.transform(pt_map, "start", timeout=rospy.Duration(1.0))
    if abs(pt_start.point.x - 0.5) > 0.08 or abs(pt_start.point.y - 0.3) > 0.08:
        raise RuntimeError(f"unexpected TF start point: x={pt_start.point.x:.3f}, y={pt_start.point.y:.3f}")

    print("PASS: start frame initialization and relative pose test")
    safe_shutdown()


if __name__ == "__main__":
    main()
