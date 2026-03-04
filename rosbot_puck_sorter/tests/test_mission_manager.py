#!/usr/bin/env python3
import threading

import actionlib
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from actionlib_msgs.msg import GoalStatus
from rosbot_puck_sorter.msg import MissionState, PickAndPlaceAction, PickAndPlaceResult
from rosbot_puck_sorter.srv import ReserveTarget, ReserveTargetResponse

from common import load_script_module, safe_shutdown, wait_for


class FakePickPlaceServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("/pick_and_place", PickAndPlaceAction, execute_cb=self._execute, auto_start=False)
        self.server.start()

    def _execute(self, _goal):
        res = PickAndPlaceResult(success=False, stage_failed="not_used", message="not used in this test")
        self.server.set_aborted(res, text="unused")


def reserve_cb(_req):
    return ReserveTargetResponse(success=False, track_id=0, color="", message="no target")


def coverage_cb(_req):
    return TriggerResponse(success=True, message="coverage skipped")


def main():
    rospy.init_node("test_mission_manager")

    rospy.set_param("~rate_hz", 20.0)
    rospy.set_param("~startup_survey_required", False)
    rospy.set_param("~home_scan_required", False)
    rospy.set_param("~single_puck_per_color", False)
    rospy.set_param("~verification_empty_time_s", 0.2)
    rospy.set_param("~empty_passes_required", 0)
    rospy.set_param("~mission_timeout_s", 5.0)

    rospy.Service("/puck_world_model/reserve_target", ReserveTarget, reserve_cb)
    rospy.Service("/coverage_search/perform_pass", Trigger, coverage_cb)
    FakePickPlaceServer()

    module = load_script_module("mission_manager.py", "mission_manager_test_mod")
    manager = module.MissionManager()

    state_msg = {"msg": None}

    def state_cb(msg):
        state_msg["msg"] = msg

    rospy.Subscriber("/mission/state", MissionState, state_cb, queue_size=10)

    t = threading.Thread(target=manager.run, daemon=True)
    t.start()

    wait_for(lambda: state_msg["msg"] is not None and state_msg["msg"].finished, timeout_s=3.0, desc="mission finished state")

    if state_msg["msg"].state != "FINISHED":
        raise RuntimeError(f"expected FINISHED state, got {state_msg['msg'].state}")

    print("PASS: mission manager completion logic test")
    safe_shutdown()


if __name__ == "__main__":
    main()
