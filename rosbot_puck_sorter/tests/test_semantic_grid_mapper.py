#!/usr/bin/env python3
import math

import rospy
from std_msgs.msg import Header

from rosbot_puck_sorter.msg import HomeBase, HomeBaseArray, PuckTrack, PuckTrackArray
from nav_msgs.msg import OccupancyGrid

from common import load_script_module, safe_shutdown, wait_for, yaw_to_quat


def idx_for_xy(grid, x, y):
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    ix = int(math.floor((x - ox) / res))
    iy = int(math.floor((y - oy) / res))
    if ix < 0 or iy < 0 or ix >= grid.info.width or iy >= grid.info.height:
        return None
    return iy * grid.info.width + ix


def main():
    rospy.init_node("test_semantic_grid_mapper")

    rospy.set_param("~start_frame", "start")
    rospy.set_param("~resolution", 0.1)
    rospy.set_param("~rect_x_min", 0.0)
    rospy.set_param("~rect_x_max", 2.0)
    rospy.set_param("~rect_y_min", 0.0)
    rospy.set_param("~rect_y_max", 2.0)
    rospy.set_param("~wall_thickness_m", 0.1)
    rospy.set_param("~home_radius_m", 0.12)
    rospy.set_param("~puck_radius_m", 0.07)
    rospy.set_param("~use_dynamic_puck_tracks", False)
    rospy.set_param("~publish_rate_hz", 5.0)

    module = load_script_module("semantic_grid_mapper.py", "semantic_grid_mapper_test_mod")
    module.SemanticGridMapper()

    pub_homes = rospy.Publisher("/startup_survey/homes_start", HomeBaseArray, queue_size=1)
    pub_pucks = rospy.Publisher("/startup_survey/pucks_start", PuckTrackArray, queue_size=1)

    out_grid = {"msg": None}
    out_homes = {"msg": None}
    out_pucks = {"msg": None}

    rospy.Subscriber("/semantic_map/grid", OccupancyGrid, lambda m: out_grid.update(msg=m), queue_size=1)
    rospy.Subscriber("/semantic_map/homes", HomeBaseArray, lambda m: out_homes.update(msg=m), queue_size=1)
    rospy.Subscriber("/semantic_map/pucks", PuckTrackArray, lambda m: out_pucks.update(msg=m), queue_size=1)

    hm = HomeBaseArray()
    hm.header = Header(stamp=rospy.Time.now(), frame_id="start")
    h = HomeBase()
    h.header = hm.header
    h.color = "green"
    h.pose_map.position.x = 0.5
    h.pose_map.position.y = 0.6
    h.pose_map.position.z = 0.0
    h.pose_map.orientation = yaw_to_quat(0.0)
    h.qr_payload = "ARUCO_1"
    h.marker_distance_m = 1.23
    hm.homes = [h]

    pm = PuckTrackArray()
    pm.header = Header(stamp=rospy.Time.now(), frame_id="start")
    p = PuckTrack()
    p.header = pm.header
    p.track_id = 7
    p.color = "red"
    p.pose_map.position.x = 1.1
    p.pose_map.position.y = 1.2
    p.pose_map.orientation = yaw_to_quat(0.0)
    p.confidence = 0.8
    p.state = "DETECTED"
    p.miss_count = 0
    p.last_seen = rospy.Time.now()
    pm.tracks = [p]

    rate = rospy.Rate(10)
    for _ in range(8):
        hm.header.stamp = rospy.Time.now()
        h.header = hm.header
        pm.header.stamp = rospy.Time.now()
        p.header = pm.header
        p.last_seen = pm.header.stamp
        pub_homes.publish(hm)
        pub_pucks.publish(pm)
        rate.sleep()

    wait_for(lambda: out_grid["msg"] is not None, timeout_s=3.0, desc="semantic map grid")
    wait_for(lambda: out_homes["msg"] is not None and len(out_homes["msg"].homes) == 1, timeout_s=3.0, desc="semantic homes")
    wait_for(lambda: out_pucks["msg"] is not None and len(out_pucks["msg"].tracks) == 1, timeout_s=3.0, desc="semantic pucks")

    grid = out_grid["msg"]
    hidx = idx_for_xy(grid, 0.5, 0.6)
    pidx = idx_for_xy(grid, 1.1, 1.2)

    if hidx is None or pidx is None:
        raise RuntimeError("home or puck index outside grid")

    if grid.data[hidx] < 80:
        raise RuntimeError(f"home cell value too low: {grid.data[hidx]}")

    if grid.data[pidx] < 60:
        raise RuntimeError(f"puck cell value too low: {grid.data[pidx]}")

    print("PASS: semantic grid mapper occupancy + object layer test")
    safe_shutdown()


if __name__ == "__main__":
    main()
