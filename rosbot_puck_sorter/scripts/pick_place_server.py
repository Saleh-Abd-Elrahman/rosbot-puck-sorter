#!/usr/bin/env python3
import math

import actionlib
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, UInt32
from std_srvs.srv import Trigger

from rosbot_puck_sorter.msg import PickAndPlaceAction, PickAndPlaceFeedback, PickAndPlaceResult, PuckTrack
from rosbot_puck_sorter.srv import SetGripper
from simple_pose_navigator import SimplePoseNavigator


class PickPlaceServer:
    def __init__(self):
        self.pregrasp_offset_m = rospy.get_param("~pregrasp_offset_m", 0.25)
        self.final_approach_m = rospy.get_param("~final_approach_m", 0.12)
        self.align_timeout_s = rospy.get_param("~align_timeout_s", 6.0)
        self.grasp_close_wait_s = rospy.get_param("~grasp_close_wait_s", 0.6)
        self.place_open_wait_s = rospy.get_param("~place_open_wait_s", 0.5)
        self.retreat_distance_m = rospy.get_param("~retreat_distance_m", 0.20)
        self.stage_retry_count = int(rospy.get_param("~stage_retry_count", 2))
        self.use_fine_align = bool(rospy.get_param("~use_fine_align", True))

        self.navigator = SimplePoseNavigator()

        rospy.wait_for_service("/gripper/set")
        self.gripper_srv = rospy.ServiceProxy("/gripper/set", SetGripper)

        self.fine_align_srv = None
        if self.use_fine_align:
            try:
                rospy.wait_for_service("/fine_align/execute", timeout=2.0)
                self.fine_align_srv = rospy.ServiceProxy("/fine_align/execute", Trigger)
            except Exception:
                rospy.logwarn("fine_align service not available; continuing without")

        self.holding_object = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.have_robot_pose = False
        rospy.Subscriber("/gripper/holding_object", Bool, self._holding_cb, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_cb, queue_size=10)

        self.pub_cmd_nav = rospy.Publisher("/cmd_vel_nav", Twist, queue_size=10)
        self.pub_delivered = rospy.Publisher("/puck_world_model/delivered_track", UInt32, queue_size=10)
        self.pub_release = rospy.Publisher("/puck_world_model/release_track", UInt32, queue_size=10)
        self.pub_current_target = rospy.Publisher("/mission/current_target", PuckTrack, queue_size=10)

        self.server = actionlib.SimpleActionServer(
            "/pick_and_place",
            PickAndPlaceAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()

        rospy.loginfo("pick_place_server ready")

    def _holding_cb(self, msg):
        self.holding_object = bool(msg.data)

    def _amcl_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.have_robot_pose = True

    def _publish_feedback(self, stage, progress):
        fb = PickAndPlaceFeedback()
        fb.stage = stage
        fb.progress = progress
        self.server.publish_feedback(fb)

    def _move_to_pose(self, x, y, yaw, timeout=45.0):
        return self.navigator.goto(x, y, yaw, timeout_s=timeout)

    def _retreat(self):
        rate = rospy.Rate(20)
        duration = max(0.1, self.retreat_distance_m / 0.08)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = -0.08
            self.pub_cmd_nav.publish(cmd)
            rate.sleep()
        self.pub_cmd_nav.publish(Twist())

    def _execute(self, goal):
        result = PickAndPlaceResult(success=False, stage_failed="", message="")

        target = PuckTrack()
        target.track_id = goal.track_id
        target.color = goal.color
        target.pose_map = goal.pick_pose
        self.pub_current_target.publish(target)

        pick_x = goal.pick_pose.position.x
        pick_y = goal.pick_pose.position.y
        place_x = goal.place_pose.position.x
        place_y = goal.place_pose.position.y

        # Stage 1: move to pre-grasp pose facing puck.
        self._publish_feedback("approach_pick", 0.10)
        if self.have_robot_pose:
            yaw_to_pick = math.atan2(pick_y - self.robot_y, pick_x - self.robot_x)
        else:
            yaw_to_pick = 0.0
        pre_x = pick_x - self.pregrasp_offset_m * math.cos(yaw_to_pick)
        pre_y = pick_y - self.pregrasp_offset_m * math.sin(yaw_to_pick)

        moved = False
        for _ in range(self.stage_retry_count + 1):
            if self._move_to_pose(pre_x, pre_y, yaw_to_pick):
                moved = True
                break
        if not moved:
            self.pub_release.publish(UInt32(data=goal.track_id))
            result.stage_failed = "approach_pick"
            result.message = "failed to reach pre-grasp"
            self.server.set_succeeded(result)
            return

        # Stage 2: fine alignment.
        self._publish_feedback("fine_align", 0.25)
        if self.fine_align_srv is not None:
            try:
                align_res = self.fine_align_srv()
                if not align_res.success:
                    rospy.logwarn("fine align failed: %s", align_res.message)
            except Exception as exc:
                rospy.logwarn("fine align call failed: %s", exc)

        # Stage 3: close gripper.
        self._publish_feedback("grasp", 0.45)
        try:
            gr = self.gripper_srv(command="close", angle_deg=0.0)
            if not gr.success:
                self.pub_release.publish(UInt32(data=goal.track_id))
                result.stage_failed = "grasp"
                result.message = "gripper close failed"
                self.server.set_succeeded(result)
                return
        except Exception as exc:
            self.pub_release.publish(UInt32(data=goal.track_id))
            result.stage_failed = "grasp"
            result.message = f"gripper close exception: {exc}"
            self.server.set_succeeded(result)
            return

        rospy.sleep(self.grasp_close_wait_s)

        # Stage 4: move to place pose.
        self._publish_feedback("navigate_home", 0.70)
        yaw_place = math.atan2(place_y - pick_y, place_x - pick_x)
        moved = False
        for _ in range(self.stage_retry_count + 1):
            if self._move_to_pose(place_x, place_y, yaw_place):
                moved = True
                break
        if not moved:
            self.pub_release.publish(UInt32(data=goal.track_id))
            result.stage_failed = "navigate_home"
            result.message = "failed to reach place pose"
            self.server.set_succeeded(result)
            return

        # Stage 5: open gripper and retreat.
        self._publish_feedback("release", 0.90)
        try:
            gr = self.gripper_srv(command="open", angle_deg=0.0)
            if not gr.success:
                result.stage_failed = "release"
                result.message = "gripper open failed"
                self.server.set_succeeded(result)
                return
        except Exception as exc:
            result.stage_failed = "release"
            result.message = f"gripper open exception: {exc}"
            self.server.set_succeeded(result)
            return

        rospy.sleep(self.place_open_wait_s)
        self._retreat()

        self.pub_delivered.publish(UInt32(data=goal.track_id))

        self._publish_feedback("completed", 1.0)
        result.success = True
        result.stage_failed = ""
        result.message = "pick-place completed"
        self.server.set_succeeded(result)


def main():
    rospy.init_node("pick_place_server")
    PickPlaceServer()
    rospy.spin()


if __name__ == "__main__":
    main()
