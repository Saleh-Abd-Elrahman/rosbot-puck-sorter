#!/usr/bin/env python3

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from common import load_script_module, safe_shutdown, wait_for


class FakeMoveBaseServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("/move_base", MoveBaseAction, execute_cb=self._execute, auto_start=False)
        self.server.start()

    def _execute(self, _goal):
        self.server.set_succeeded(MoveBaseResult())


def main():
    rospy.init_node("test_coverage_search")

    rospy.set_param("~rect_x_min", 0.0)
    rospy.set_param("~rect_x_max", 1.0)
    rospy.set_param("~rect_y_min", 0.0)
    rospy.set_param("~rect_y_max", 0.8)
    rospy.set_param("~border_margin_m", 0.1)
    rospy.set_param("~lane_spacing_m", 0.4)
    rospy.set_param("~scan_dwell_s", 0.01)

    FakeMoveBaseServer()

    module = load_script_module("coverage_search.py", "coverage_search_test_mod")
    module.CoverageSearch()

    pass_count = {"value": -1}

    def pass_cb(msg):
        pass_count["value"] = int(msg.data)

    rospy.Subscriber("/coverage_search/pass_count", Int32, pass_cb, queue_size=10)

    rospy.wait_for_service("/coverage_search/perform_pass", timeout=5.0)
    srv = rospy.ServiceProxy("/coverage_search/perform_pass", Trigger)
    rsp = srv()
    if not rsp.success:
        raise RuntimeError(f"coverage pass failed: {rsp.message}")

    wait_for(lambda: pass_count["value"] >= 1, timeout_s=2.0, desc="coverage pass count increment")

    print("PASS: coverage search pass test")
    safe_shutdown()


if __name__ == "__main__":
    main()
