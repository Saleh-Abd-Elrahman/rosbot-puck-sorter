#!/usr/bin/env python3
import math
import os
import sys
import threading

import actionlib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt32
from std_srvs.srv import Trigger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from rosbot_puck_sorter.msg import (
    PickAndPlaceAction,
    PickAndPlaceFeedback,
    PickAndPlaceResult,
    PuckDetectionArray,
    PuckTrack,
)
from rosbot_puck_sorter.srv import SetGripper
from simple_pose_navigator import SimplePoseNavigator


class PickPlaceServer:
    def __init__(self):
        self.lock = threading.Lock()

        self.pregrasp_offset_m = rospy.get_param("~pregrasp_offset_m", 0.25)
        self.final_approach_m = rospy.get_param("~final_approach_m", 0.12)
        self.final_approach_speed_m_s = rospy.get_param("~final_approach_speed_m_s", 0.08)
        self.grasp_close_wait_s = rospy.get_param("~grasp_close_wait_s", 0.6)
        self.place_open_wait_s = rospy.get_param("~place_open_wait_s", 0.5)
        self.retreat_distance_m = rospy.get_param("~retreat_distance_m", 0.20)
        self.stage_retry_count = int(rospy.get_param("~stage_retry_count", 2))
        self.use_fine_align = bool(rospy.get_param("~use_fine_align", True))
        self.release_verify_timeout_s = float(rospy.get_param("~release_verify_timeout_s", 1.0))
        self.pickup_verify_mode = str(rospy.get_param("~pickup_verify_mode", "vision_plus_servo")).strip().lower()
        self.pickup_verify_timeout_s = float(rospy.get_param("~pickup_verify_timeout_s", 1.5))
        self.pickup_verify_clear_hold_s = float(rospy.get_param("~pickup_verify_clear_hold_s", 0.25))
        self.pickup_verify_detection_stale_s = float(rospy.get_param("~pickup_verify_detection_stale_s", 0.6))
        self.pickup_verify_map_radius_m = float(rospy.get_param("~pickup_verify_map_radius_m", 0.18))
        self.pickup_verify_confidence_min = float(rospy.get_param("~pickup_verify_confidence_min", 0.35))
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

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
        self.holding_stamp = rospy.Time(0)
        self.latest_detections = []
        self.latest_detections_stamp = rospy.Time(0)
        rospy.Subscriber("/gripper/holding_object", Bool, self._holding_cb, queue_size=10)
        rospy.Subscriber("/puck/detections", PuckDetectionArray, self._detections_cb, queue_size=10)

        self.pub_cmd_nav = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.pub_delivered = rospy.Publisher("/puck_world_model/delivered_track", UInt32, queue_size=10)
        self.pub_release = rospy.Publisher("/puck_world_model/release_track", UInt32, queue_size=10)
        self.pub_lost = rospy.Publisher("/puck_world_model/lost_track", UInt32, queue_size=10)
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
        with self.lock:
            self.holding_object = bool(msg.data)
            self.holding_stamp = rospy.Time.now()

    def _detections_cb(self, msg):
        with self.lock:
            self.latest_detections = list(msg.detections)
            self.latest_detections_stamp = rospy.Time.now()

    def _publish_feedback(self, stage, progress):
        fb = PickAndPlaceFeedback()
        fb.stage = stage
        fb.progress = progress
        self.server.publish_feedback(fb)

    def _move_to_pose(self, x, y, yaw, timeout=45.0):
        return self.navigator.goto(x, y, yaw, timeout_s=timeout)

    def _drive_straight(self, distance_m, speed_m_s):
        distance_m = float(distance_m)
        if abs(distance_m) <= 1e-6:
            return True

        speed = max(0.01, abs(float(speed_m_s)))
        direction = 1.0 if distance_m >= 0.0 else -1.0
        duration = abs(distance_m) / speed
        rate = rospy.Rate(20)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = direction * speed
            self.pub_cmd_nav.publish(cmd)
            rate.sleep()
        self.pub_cmd_nav.publish(Twist())
        return not rospy.is_shutdown()

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

    def _recent_holding(self):
        with self.lock:
            holding = self.holding_object
            stamp = self.holding_stamp
        if not holding or stamp == rospy.Time(0):
            return False
        return (rospy.Time.now() - stamp).to_sec() <= self.pickup_verify_timeout_s

    def _target_visible_at_pick(self, goal):
        with self.lock:
            detections = list(self.latest_detections)
            stamp = self.latest_detections_stamp

        if stamp == rospy.Time(0):
            return None
        if (rospy.Time.now() - stamp).to_sec() > self.pickup_verify_detection_stale_s:
            return None

        color = str(goal.color).strip().lower()
        pick = goal.pick_pose.position
        for det in detections:
            if color and str(det.color).strip().lower() != color:
                continue
            if det.confidence < self.pickup_verify_confidence_min:
                continue
            dx = det.position_map.x - pick.x
            dy = det.position_map.y - pick.y
            if math.hypot(dx, dy) <= self.pickup_verify_map_radius_m:
                return True
        return False

    def _verify_pickup(self, goal, target_visible_at_grasp):
        mode = self.pickup_verify_mode
        if mode in ("", "none", "off", "disabled"):
            return True, "pickup verification disabled"

        use_servo = mode in ("servo", "servo_only", "vision_plus_servo")
        use_vision = mode in ("vision", "vision_only", "vision_plus_servo")
        if not use_servo and not use_vision:
            return False, f"unsupported pickup_verify_mode '{mode}'"

        deadline = rospy.Time.now() + rospy.Duration(max(0.05, self.pickup_verify_timeout_s))
        missing_since = None
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now = rospy.Time.now()
            if use_servo and self._recent_holding():
                return True, "gripper hold confirmed"

            if use_vision:
                visible = self._target_visible_at_pick(goal)
                if visible is True:
                    missing_since = None
                elif visible is False and target_visible_at_grasp:
                    if missing_since is None:
                        missing_since = now
                    if (now - missing_since).to_sec() >= self.pickup_verify_clear_hold_s:
                        return True, "target no longer visible at pick pose"

            rate.sleep()

        if use_vision and not target_visible_at_grasp and not use_servo:
            return False, "pickup not verified: target was not visible at grasp pose"
        if use_servo and not use_vision:
            return False, "pickup not verified: gripper hold not confirmed"
        return False, "pickup not verified by gripper or vision"

    def _open_gripper_best_effort(self):
        try:
            gr = self.gripper_srv(command="open", angle_deg=0.0)
            if not gr.success:
                rospy.logwarn("best-effort gripper open failed: %s", gr.message)
                return False
            return True
        except Exception as exc:
            rospy.logwarn("failed to reopen gripper after pickup failure: %s", exc)
            return False

    def _mark_track_lost(self, track_id):
        self.pub_lost.publish(UInt32(data=track_id))

    def _wait_for_release_confirmation(self, min_stamp):
        if self.release_verify_timeout_s <= 0.0:
            return True, "release verification disabled"

        deadline = rospy.Time.now() + rospy.Duration(max(0.05, self.release_verify_timeout_s))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                holding = self.holding_object
                stamp = self.holding_stamp
            fresh = stamp != rospy.Time(0) and (stamp - min_stamp).to_sec() >= 0.0
            if fresh and not holding:
                return True, "release confirmed"
            rate.sleep()

        return False, "release not confirmed by gripper state"

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
        have_robot_pose, robot_x, robot_y, _robot_yaw = self.navigator.current_pose()
        if have_robot_pose:
            yaw_to_pick = math.atan2(pick_y - robot_y, pick_x - robot_x)
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

        if self.final_approach_m > 0.0:
            self._publish_feedback("final_approach", 0.35)
            if not self._drive_straight(self.final_approach_m, self.final_approach_speed_m_s):
                self.pub_release.publish(UInt32(data=goal.track_id))
                result.stage_failed = "final_approach"
                result.message = "shutdown during final approach"
                self.server.set_succeeded(result)
                return

        target_visible_at_grasp = self._target_visible_at_pick(goal) is True

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

        verified, verify_msg = self._verify_pickup(goal, target_visible_at_grasp)
        if not verified:
            self._open_gripper_best_effort()
            self.pub_release.publish(UInt32(data=goal.track_id))
            result.stage_failed = "verify_pickup"
            result.message = verify_msg
            self.server.set_succeeded(result)
            return

        # Stage 4: move to place pose.
        self._publish_feedback("navigate_home", 0.70)
        yaw_place = math.atan2(place_y - pick_y, place_x - pick_x)
        moved = False
        for _ in range(self.stage_retry_count + 1):
            if self._move_to_pose(place_x, place_y, yaw_place):
                moved = True
                break
        if not moved:
            self._open_gripper_best_effort()
            self._mark_track_lost(goal.track_id)
            result.stage_failed = "navigate_home"
            result.message = "failed to reach place pose"
            self.server.set_succeeded(result)
            return

        # Stage 5: open gripper and retreat.
        self._publish_feedback("release", 0.90)
        release_started = rospy.Time.now()
        try:
            gr = self.gripper_srv(command="open", angle_deg=0.0)
            if not gr.success:
                self._mark_track_lost(goal.track_id)
                result.stage_failed = "release"
                result.message = "gripper open failed"
                self.server.set_succeeded(result)
                return
        except Exception as exc:
            self._mark_track_lost(goal.track_id)
            result.stage_failed = "release"
            result.message = f"gripper open exception: {exc}"
            self.server.set_succeeded(result)
            return

        rospy.sleep(self.place_open_wait_s)

        released, release_msg = self._wait_for_release_confirmation(release_started)
        if not released:
            self._mark_track_lost(goal.track_id)
            result.stage_failed = "release_verify"
            result.message = release_msg
            self.server.set_succeeded(result)
            return

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
