#!/usr/bin/env python3
import time

import actionlib
import rospy
from std_msgs.msg import Bool, Int32, UInt32
from std_srvs.srv import Trigger

from rosbot_puck_sorter.msg import HomeBaseArray, MissionState, PickAndPlaceAction, PickAndPlaceGoal, PuckTrackArray
from rosbot_puck_sorter.srv import ReserveTarget, ScanHomes


class MissionManager:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate_hz", 10.0)
        self.target_policy = rospy.get_param("~target_policy", "nearest_reachable")
        self.max_pick_attempts_per_track = int(rospy.get_param("~max_pick_attempts_per_track", 3))
        self.blacklist_duration_s = rospy.get_param("~blacklist_duration_s", 20.0)
        self.verification_empty_time_s = rospy.get_param("~verification_empty_time_s", 20.0)
        self.empty_passes_required = int(rospy.get_param("~empty_passes_required", 2))
        self.mission_timeout_s = rospy.get_param("~mission_timeout_s", 1800.0)
        self.home_scan_required = bool(rospy.get_param("~home_scan_required", True))
        self.track_confidence_min = rospy.get_param("~track_confidence_min", 0.55)

        self.homes = {}
        self.tracks = {}
        self.coverage_pass_count = 0
        self.holding_object = False

        self.attempt_counts = {}
        self.blacklist_until = {}

        self.state = "INIT"
        self.note = ""
        self.active_track_id = 0
        self.goal_in_flight = False
        self.last_empty_start = None
        self.started_at = rospy.Time.now()

        self.pub_state = rospy.Publisher("/mission/state", MissionState, queue_size=10, latch=True)
        self.pub_release = rospy.Publisher("/puck_world_model/release_track", UInt32, queue_size=10)

        rospy.Subscriber("/home_bases", HomeBaseArray, self._homes_cb, queue_size=1)
        rospy.Subscriber("/puck/tracks", PuckTrackArray, self._tracks_cb, queue_size=10)
        rospy.Subscriber("/coverage_search/pass_count", Int32, self._pass_cb, queue_size=10)
        rospy.Subscriber("/gripper/holding_object", Bool, self._holding_cb, queue_size=10)

        rospy.wait_for_service("/puck_world_model/reserve_target")
        self.reserve_srv = rospy.ServiceProxy("/puck_world_model/reserve_target", ReserveTarget)

        self.scan_srv = None
        if self.home_scan_required:
            rospy.wait_for_service("/scan_homes")
            self.scan_srv = rospy.ServiceProxy("/scan_homes", ScanHomes)

        rospy.wait_for_service("/coverage_search/perform_pass")
        self.coverage_srv = rospy.ServiceProxy("/coverage_search/perform_pass", Trigger)

        self.pick_client = actionlib.SimpleActionClient("/pick_and_place", PickAndPlaceAction)
        rospy.loginfo("Waiting for /pick_and_place action server...")
        self.pick_client.wait_for_server(rospy.Duration(10.0))

        rospy.loginfo("mission_manager ready")

    def _homes_cb(self, msg):
        self.homes = {h.color: h.pose_map for h in msg.homes}

    def _tracks_cb(self, msg):
        self.tracks = {t.track_id: t for t in msg.tracks}

    def _pass_cb(self, msg):
        self.coverage_pass_count = msg.data

    def _holding_cb(self, msg):
        self.holding_object = bool(msg.data)

    def _publish_state(self, finished=False):
        remaining = 0
        for tr in self.tracks.values():
            if tr.state == "DETECTED" and tr.confidence >= self.track_confidence_min:
                remaining += 1

        out = MissionState()
        out.header.stamp = rospy.Time.now()
        out.state = self.state
        out.active_track_id = int(self.active_track_id)
        out.remaining_estimate = int(remaining)
        out.finished = bool(finished)
        out.note = self.note
        self.pub_state.publish(out)

    def _is_blacklisted(self, track_id):
        until = self.blacklist_until.get(track_id, 0.0)
        return time.time() < until

    def _handle_action_result(self):
        result = self.pick_client.get_result()
        if result is None:
            return

        if result.success:
            self.note = f"track {self.active_track_id} delivered"
        else:
            self.note = f"track {self.active_track_id} failed at {result.stage_failed}"
            self.attempt_counts[self.active_track_id] = self.attempt_counts.get(self.active_track_id, 0) + 1
            if self.attempt_counts[self.active_track_id] >= self.max_pick_attempts_per_track:
                self.blacklist_until[self.active_track_id] = time.time() + self.blacklist_duration_s
                self.note += " (temporarily blacklisted)"

        self.active_track_id = 0
        self.goal_in_flight = False

    def _done_condition(self):
        remaining = 0
        for tr in self.tracks.values():
            if tr.state == "DETECTED" and tr.confidence >= self.track_confidence_min:
                if not self._is_blacklisted(tr.track_id):
                    remaining += 1

        if remaining == 0 and not self.holding_object and not self.goal_in_flight:
            if self.last_empty_start is None:
                self.last_empty_start = rospy.Time.now()
            empty_for = (rospy.Time.now() - self.last_empty_start).to_sec()
            if empty_for >= self.verification_empty_time_s and self.coverage_pass_count >= self.empty_passes_required:
                return True
        else:
            self.last_empty_start = None

        return False

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        if self.home_scan_required:
            self.state = "SCAN_HOMES"
            self.note = "scanning QR homes"
            self._publish_state()
            try:
                rsp = self.scan_srv(start=True)
                if not rsp.success:
                    self.state = "ERROR"
                    self.note = f"home scan failed: {rsp.message}"
                    self._publish_state()
                    return
            except Exception as exc:
                self.state = "ERROR"
                self.note = f"home scan exception: {exc}"
                self._publish_state()
                return

        self.state = "RUNNING"

        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.started_at).to_sec() > self.mission_timeout_s:
                self.state = "TIMEOUT"
                self.note = "mission timeout reached"
                self._publish_state()
                return

            if self.goal_in_flight:
                if self.pick_client.wait_for_result(rospy.Duration(0.0)):
                    self._handle_action_result()
                self._publish_state()
                rate.sleep()
                continue

            if self._done_condition():
                self.state = "FINISHED"
                self.note = "all pucks sorted"
                self._publish_state(finished=True)
                return

            # Reserve a target from the world model.
            try:
                reserve_rsp = self.reserve_srv(strategy=self.target_policy)
            except Exception as exc:
                self.note = f"reserve service error: {exc}"
                self._publish_state()
                rate.sleep()
                continue

            if reserve_rsp.success and reserve_rsp.track_id > 0:
                if self._is_blacklisted(reserve_rsp.track_id):
                    self.pub_release.publish(UInt32(data=reserve_rsp.track_id))
                    self.note = f"reserved track {reserve_rsp.track_id} is blacklisted, waiting"
                    self._publish_state()
                    rate.sleep()
                    continue

                home_pose = self.homes.get(reserve_rsp.color)
                if home_pose is None:
                    self.pub_release.publish(UInt32(data=reserve_rsp.track_id))
                    self.note = f"home for color {reserve_rsp.color} unavailable"
                    self._publish_state()
                    rate.sleep()
                    continue

                goal = PickAndPlaceGoal()
                goal.track_id = reserve_rsp.track_id
                goal.color = reserve_rsp.color
                goal.pick_pose = reserve_rsp.pose_map
                goal.place_pose = home_pose

                self.pick_client.send_goal(goal)
                self.active_track_id = reserve_rsp.track_id
                self.goal_in_flight = True
                self.note = f"executing pick-place for track {self.active_track_id}"
                self._publish_state()
                rate.sleep()
                continue

            # No target available: run one coverage pass to discover pucks.
            self.state = "SEARCH"
            self.note = "no target available, performing coverage pass"
            self._publish_state()
            try:
                rsp = self.coverage_srv()
                self.note = rsp.message
            except Exception as exc:
                self.note = f"coverage service error: {exc}"

            self.state = "RUNNING"
            self._publish_state()
            rate.sleep()


def main():
    rospy.init_node("mission_manager")
    manager = MissionManager()
    manager.run()


if __name__ == "__main__":
    main()
