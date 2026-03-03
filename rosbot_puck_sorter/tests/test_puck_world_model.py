#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header, UInt32

from rosbot_puck_sorter.msg import PuckDetection, PuckDetectionArray, PuckTrackArray
from rosbot_puck_sorter.srv import ReserveTarget

from common import load_script_module, safe_shutdown, wait_for


def main():
    rospy.init_node("test_puck_world_model")

    rospy.set_param("~conf_init", 0.70)
    rospy.set_param("~conf_valid_min", 0.55)
    rospy.set_param("~publish_start_relative", False)
    rospy.set_param("~update_rate_hz", 20.0)

    module = load_script_module("puck_world_model.py", "puck_world_model_test_mod")
    module.PuckWorldModel()

    pub_det = rospy.Publisher("/puck/detections", PuckDetectionArray, queue_size=10)
    pub_amcl = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size=10)
    pub_delivered = rospy.Publisher("/puck_world_model/delivered_track", UInt32, queue_size=10)

    latest_tracks = {"msg": None}

    def tracks_cb(msg):
        latest_tracks["msg"] = msg

    rospy.Subscriber("/puck/tracks", PuckTrackArray, tracks_cb, queue_size=10)

    amcl = PoseWithCovarianceStamped()
    amcl.header.frame_id = "map"
    amcl.pose.pose.position.x = 0.0
    amcl.pose.pose.position.y = 0.0
    amcl.pose.pose.orientation.w = 1.0

    det_array = PuckDetectionArray()
    det_array.header = Header(stamp=rospy.Time.now(), frame_id="map")
    det = PuckDetection()
    det.header = det_array.header
    det.color = "red"
    det.position_camera = Point(x=1.0, y=0.0, z=1.0)
    det.position_map = Point(x=1.2, y=0.3, z=0.0)
    det.confidence = 0.9
    det.radius_m = 0.03
    det.contour_area_px = 200
    det_array.detections = [det]

    rate = rospy.Rate(15)
    for _ in range(20):
        amcl.header.stamp = rospy.Time.now()
        det_array.header.stamp = rospy.Time.now()
        det.header.stamp = det_array.header.stamp
        pub_amcl.publish(amcl)
        pub_det.publish(det_array)
        rate.sleep()

    wait_for(
        lambda: latest_tracks["msg"] is not None and any(t.color == "red" and t.state == "DETECTED" for t in latest_tracks["msg"].tracks),
        timeout_s=3.0,
        desc="red track in world model",
    )

    rospy.wait_for_service("/puck_world_model/reserve_target", timeout=5.0)
    reserve_srv = rospy.ServiceProxy("/puck_world_model/reserve_target", ReserveTarget)
    rsp = reserve_srv(strategy="nearest_reachable")
    if not rsp.success:
        raise RuntimeError(f"reserve_target failed: {rsp.message}")
    if rsp.color != "red":
        raise RuntimeError(f"reserved wrong color: {rsp.color}")

    pub_delivered.publish(UInt32(data=rsp.track_id))

    wait_for(
        lambda: latest_tracks["msg"] is not None and any(t.track_id == rsp.track_id and t.state == "DELIVERED" for t in latest_tracks["msg"].tracks),
        timeout_s=3.0,
        desc="track delivered state",
    )

    print("PASS: puck world model reserve/deliver test")
    safe_shutdown()


if __name__ == "__main__":
    main()
