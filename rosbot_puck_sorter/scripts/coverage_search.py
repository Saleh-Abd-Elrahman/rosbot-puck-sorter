#!/usr/bin/env python3
import math
import os
import sys

import rospy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from simple_pose_navigator import SimplePoseNavigator


class CoverageSearch:
    def __init__(self):
        self.rect_x_min = rospy.get_param("~rect_x_min", 0.0)
        self.rect_x_max = rospy.get_param("~rect_x_max", 5.0)
        self.rect_y_min = rospy.get_param("~rect_y_min", 0.0)
        self.rect_y_max = rospy.get_param("~rect_y_max", 4.0)
        self.border_margin_m = rospy.get_param("~border_margin_m", 0.25)
        self.lane_spacing_m = rospy.get_param("~lane_spacing_m", 0.45)
        self.scan_dwell_s = rospy.get_param("~scan_dwell_s", 1.0)
        self.use_fixed_yaw = bool(rospy.get_param("~use_fixed_yaw", False))
        self.yaw_at_waypoint_rad = rospy.get_param("~yaw_at_waypoint_rad", 0.0)

        self.pub_status = rospy.Publisher("/coverage_search/status", String, queue_size=10)
        self.pub_passes = rospy.Publisher("/coverage_search/pass_count", Int32, queue_size=10, latch=True)

        self.navigator = SimplePoseNavigator()

        self.pass_count = 0
        self.waypoints = self._build_waypoints()
        self.pub_passes.publish(Int32(data=self.pass_count))

        rospy.Service("/coverage_search/perform_pass", Trigger, self._perform_pass)
        rospy.loginfo("coverage_search ready with %d waypoints", len(self.waypoints))

    def _build_waypoints(self):
        x0 = self.rect_x_min + self.border_margin_m
        x1 = self.rect_x_max - self.border_margin_m
        y0 = self.rect_y_min + self.border_margin_m
        y1 = self.rect_y_max - self.border_margin_m

        if x1 <= x0 or y1 <= y0:
            return []

        ys = []
        y = y0
        while y <= y1 + 1e-6:
            ys.append(y)
            y += self.lane_spacing_m

        wps = []
        for i, yy in enumerate(ys):
            if i % 2 == 0:
                wps.append((x0, yy, 0.0))
                wps.append((x1, yy, math.pi))
            else:
                wps.append((x1, yy, math.pi))
                wps.append((x0, yy, 0.0))

        return wps

    def _goto(self, x, y, yaw):
        return self.navigator.goto(x, y, yaw, timeout_s=40.0)

    def _perform_pass(self, _req):
        if not self.waypoints:
            return TriggerResponse(success=False, message="invalid coverage bounds; no waypoints")

        self.pub_status.publish(String(data="running"))
        for x, y, yaw in self.waypoints:
            if rospy.is_shutdown():
                return TriggerResponse(success=False, message="shutdown")
            cmd_yaw = self.yaw_at_waypoint_rad if self.use_fixed_yaw else yaw
            if not self._goto(x, y, cmd_yaw):
                self.pub_status.publish(String(data="nav_failure"))
                return TriggerResponse(success=False, message="cmd_vel navigation failed during coverage pass")
            rospy.sleep(self.scan_dwell_s)

        self.pass_count += 1
        self.pub_passes.publish(Int32(data=self.pass_count))
        self.pub_status.publish(String(data="completed"))
        return TriggerResponse(success=True, message=f"coverage pass completed: {self.pass_count}")


def main():
    rospy.init_node("coverage_search")
    CoverageSearch()
    rospy.spin()


if __name__ == "__main__":
    main()
