#!/usr/bin/env python3

import actionlib
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger, TriggerResponse

from rosbot_puck_sorter.msg import PickAndPlaceAction, PickAndPlaceGoal
from rosbot_puck_sorter.srv import SetGripper, SetGripperResponse

from common import load_script_module, safe_shutdown, wait_for


class FakeNavigator:
    def __init__(self):
        self.goals = []

    def goto(self, x, y, yaw, timeout_s=45.0):
        self.goals.append((x, y, yaw, timeout_s))
        return True


def gripper_cb(req):
    return SetGripperResponse(success=True, message=f"ok:{req.command}")


def fine_align_cb(_req):
    return TriggerResponse(success=True, message="aligned")


def main():
    rospy.init_node("test_pick_place_server")

    rospy.set_param("~pregrasp_offset_m", 0.15)
    rospy.set_param("~final_approach_m", 0.0)
    rospy.set_param("~grasp_close_wait_s", 0.01)
    rospy.set_param("~place_open_wait_s", 0.01)
    rospy.set_param("~release_verify_timeout_s", 0.0)
    rospy.set_param("~retreat_distance_m", 0.05)
    rospy.set_param("~stage_retry_count", 0)
    rospy.set_param("~use_fine_align", True)
    rospy.set_param("~pickup_verify_mode", "none")

    rospy.Service("/gripper/set", SetGripper, gripper_cb)
    rospy.Service("/fine_align/execute", Trigger, fine_align_cb)

    module = load_script_module("pick_place_server.py", "pick_place_server_test_mod")
    module.SimplePoseNavigator = FakeNavigator
    module.PickPlaceServer()

    delivered = {"track_id": 0}

    def delivered_cb(msg):
        delivered["track_id"] = int(msg.data)

    rospy.Subscriber("/puck_world_model/delivered_track", UInt32, delivered_cb, queue_size=10)

    client = actionlib.SimpleActionClient("/pick_and_place", PickAndPlaceAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        raise RuntimeError("/pick_and_place action server unavailable")

    goal = PickAndPlaceGoal()
    goal.track_id = 42
    goal.color = "red"

    pick_pose = Pose()
    pick_pose.position.x = 1.0
    pick_pose.position.y = 0.5
    pick_pose.orientation.w = 1.0

    place_pose = Pose()
    place_pose.position.x = 0.2
    place_pose.position.y = 0.2
    place_pose.orientation.w = 1.0

    goal.pick_pose = pick_pose
    goal.place_pose = place_pose

    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(8.0)):
        raise RuntimeError("pick_and_place action timed out")

    result = client.get_result()
    if result is None or not result.success:
        raise RuntimeError(f"pick_place failed: {result}")

    wait_for(lambda: delivered["track_id"] == 42, timeout_s=2.0, desc="delivered track publish")

    print("PASS: pick_place_server action test")
    safe_shutdown()


if __name__ == "__main__":
    main()
