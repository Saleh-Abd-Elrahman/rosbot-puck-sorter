#!/usr/bin/env python3
import math
import time

import actionlib
import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Bool, Int32, UInt32
from std_srvs.srv import Trigger

from rosbot_puck_sorter.msg import HomeBase, HomeBaseArray, MissionState, PickAndPlaceAction, PickAndPlaceGoal, PuckTrackArray
from rosbot_puck_sorter.srv import ReserveTarget, ScanHomes


def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MissionManager:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate_hz", 10.0)

        self.startup_survey_required = bool(rospy.get_param("~startup_survey_required", True))
        self.startup_survey_must_succeed = bool(rospy.get_param("~startup_survey_must_succeed", True))
        self.validate_startup_data = bool(rospy.get_param("~validate_startup_data", True))
        self.min_startup_homes = int(rospy.get_param("~min_startup_homes", 3))
        self.startup_validation_timeout_s = float(rospy.get_param("~startup_validation_timeout_s", 2.5))
        self.skip_corner_scan_if_startup_valid = bool(rospy.get_param("~skip_corner_scan_if_startup_valid", True))

        self.target_policy = rospy.get_param("~target_policy", "cost_optimal")
        self.single_puck_per_color = bool(rospy.get_param("~single_puck_per_color", True))
        self.expected_colors = self._normalize_color_list(rospy.get_param("~expected_colors", ["red", "green", "blue"]))
        self.require_all_expected_colors = bool(rospy.get_param("~require_all_expected_colors", True))
        self.max_pick_attempts_per_track = int(rospy.get_param("~max_pick_attempts_per_track", 3))
        self.blacklist_duration_s = rospy.get_param("~blacklist_duration_s", 20.0)
        self.verification_empty_time_s = rospy.get_param("~verification_empty_time_s", 20.0)
        self.empty_passes_required = int(rospy.get_param("~empty_passes_required", 2))
        self.mission_timeout_s = rospy.get_param("~mission_timeout_s", 1800.0)
        self.home_scan_required = bool(rospy.get_param("~home_scan_required", True))
        self.track_confidence_min = rospy.get_param("~track_confidence_min", 0.55)
        self.gripper_holding_topic = rospy.get_param("~gripper_holding_topic", "/gripper/holding_object")
        self.search_trigger_s = float(rospy.get_param("~search_trigger_s", 4.0))

        self.use_make_plan_cost = bool(rospy.get_param("~use_make_plan_cost", True))
        self.make_plan_service_name = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        self.make_plan_tolerance = float(rospy.get_param("~make_plan_tolerance", 0.1))

        self.lookahead_top_k = int(rospy.get_param("~lookahead_top_k", 4))
        self.lookahead_weight = float(rospy.get_param("~lookahead_weight", 0.6))
        self.confidence_penalty_weight = float(rospy.get_param("~confidence_penalty_weight", 0.8))
        self.attempt_penalty_weight = float(rospy.get_param("~attempt_penalty_weight", 0.8))
        self.wall_penalty_weight = float(rospy.get_param("~wall_penalty_weight", 0.8))
        self.wall_penalty_distance_m = float(rospy.get_param("~wall_penalty_distance_m", 0.25))
        self.turn_penalty_weight = float(rospy.get_param("~turn_penalty_weight", 0.25))

        self.rect_x_min = float(rospy.get_param("~rect_x_min", 0.0))
        self.rect_x_max = float(rospy.get_param("~rect_x_max", 5.0))
        self.rect_y_min = float(rospy.get_param("~rect_y_min", 0.0))
        self.rect_y_max = float(rospy.get_param("~rect_y_max", 4.0))

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.pose_source = str(rospy.get_param("~pose_source", rospy.get_param("~nav_pose_source", "auto"))).strip().lower()
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")

        self.homes = {}
        self.tracks = {}
        self.startup_homes_msg = None
        self.startup_pucks_msg = None

        self.coverage_pass_count = 0
        self.holding_object = False

        self.have_robot_pose = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.attempt_counts = {}
        self.blacklist_until = {}
        self.completed_colors = set()

        self.state = "INIT"
        self.note = ""
        self.active_track_id = 0
        self.active_track_color = ""
        self.goal_in_flight = False
        self.last_empty_start = None
        self.no_target_since = None
        self.started_at = rospy.Time.now()

        self.pub_state = rospy.Publisher("/mission/state", MissionState, queue_size=10, latch=True)
        self.pub_release = rospy.Publisher("/puck_world_model/release_track", UInt32, queue_size=10)
        self.pub_homes_seed = rospy.Publisher("/home_bases", HomeBaseArray, queue_size=1, latch=True)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/home_bases", HomeBaseArray, self._homes_cb, queue_size=1)
        rospy.Subscriber("/startup_survey/homes_start", HomeBaseArray, self._startup_homes_cb, queue_size=1)
        rospy.Subscriber("/startup_survey/pucks_start", PuckTrackArray, self._startup_pucks_cb, queue_size=1)
        rospy.Subscriber("/puck/tracks", PuckTrackArray, self._tracks_cb, queue_size=10)
        rospy.Subscriber("/coverage_search/pass_count", Int32, self._pass_cb, queue_size=10)
        rospy.Subscriber(self.gripper_holding_topic, Bool, self._holding_cb, queue_size=10)
        if self.pose_source in ("auto", "amcl", "pose", "pose_topic"):
            rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self._pose_cb, queue_size=1)
        if self.pose_source in ("auto", "odom"):
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        self.pose_timer = rospy.Timer(rospy.Duration(0.25), self._pose_timer_cb)

        rospy.wait_for_service("/puck_world_model/reserve_target")
        self.reserve_srv = rospy.ServiceProxy("/puck_world_model/reserve_target", ReserveTarget)

        self.scan_srv = None
        if self.home_scan_required:
            rospy.wait_for_service("/scan_homes")
            self.scan_srv = rospy.ServiceProxy("/scan_homes", ScanHomes)

        self.startup_survey_srv = None
        if self.startup_survey_required:
            rospy.wait_for_service("/startup_survey/run")
            self.startup_survey_srv = rospy.ServiceProxy("/startup_survey/run", Trigger)

        rospy.wait_for_service("/coverage_search/perform_pass")
        self.coverage_srv = rospy.ServiceProxy("/coverage_search/perform_pass", Trigger)

        self.make_plan_srv = None
        if self.use_make_plan_cost:
            try:
                rospy.wait_for_service(self.make_plan_service_name, timeout=0.3)
                self.make_plan_srv = rospy.ServiceProxy(self.make_plan_service_name, GetPlan)
                rospy.loginfo("mission_manager using %s for path cost", self.make_plan_service_name)
            except Exception:
                self.make_plan_srv = None
                self.use_make_plan_cost = False
                rospy.logwarn("mission_manager path-cost fallback: make_plan unavailable")

        self.pick_client = actionlib.SimpleActionClient("/pick_and_place", PickAndPlaceAction)
        rospy.loginfo("Waiting for /pick_and_place action server...")
        self.pick_client.wait_for_server(rospy.Duration(10.0))

        rospy.loginfo("mission_manager ready")

    @staticmethod
    def _normalize_color(color):
        return str(color).strip().lower()

    @classmethod
    def _normalize_color_list(cls, values):
        if values is None:
            return []
        if isinstance(values, str):
            values = [v.strip() for v in values.split(",")]
        out = []
        for v in values:
            c = cls._normalize_color(v)
            if c and c not in out:
                out.append(c)
        return out

    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _set_robot_pose(self, x, y, yaw):
        self.robot_x = float(x)
        self.robot_y = float(y)
        self.robot_yaw = float(yaw)
        self.have_robot_pose = True

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self._set_robot_pose(p.position.x, p.position.y, self._yaw_from_quat(p.orientation))

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self._set_robot_pose(p.position.x, p.position.y, self._yaw_from_quat(p.orientation))

    def _pose_timer_cb(self, _event):
        if self.pose_source in ("open_loop", "cmd_vel", "cmd_vel_only"):
            return

        frames = [(self.map_frame, self.base_frame)]
        if self.map_frame != self.odom_frame:
            frames.append((self.odom_frame, self.base_frame))

        for target_frame, source_frame in frames:
            if self.pose_source == "odom" and target_frame != self.odom_frame:
                continue
            if self.pose_source in ("amcl", "pose", "pose_topic") and target_frame != self.map_frame:
                continue
            if self._update_pose_from_tf(target_frame, source_frame):
                return

    def _update_pose_from_tf(self, target_frame, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), timeout=rospy.Duration(0.02))
        except Exception:
            return False

        self._set_robot_pose(t.transform.translation.x, t.transform.translation.y, self._yaw_from_quat(t.transform.rotation))
        return True

    def _homes_cb(self, msg):
        homes = {}
        for h in msg.homes:
            homes[self._normalize_color(h.color)] = h.pose_map
        self.homes = homes

    def _startup_homes_cb(self, msg):
        self.startup_homes_msg = msg

    def _startup_pucks_cb(self, msg):
        self.startup_pucks_msg = msg

    def _tracks_cb(self, msg):
        self.tracks = {t.track_id: t for t in msg.tracks}

    def _pass_cb(self, msg):
        self.coverage_pass_count = msg.data

    def _holding_cb(self, msg):
        self.holding_object = bool(msg.data)

    def _remaining_viable_count(self):
        candidates = self._build_candidates()
        if self.single_puck_per_color:
            pending = self._pending_expected_colors()
            if pending:
                return len(pending)
            return len({c["color"] for c in candidates})
        return len(candidates)

    def _pending_expected_colors(self):
        if not self.expected_colors:
            return []
        return [c for c in self.expected_colors if c not in self.completed_colors]

    def _publish_state(self, finished=False):
        out = MissionState()
        out.header.stamp = rospy.Time.now()
        out.state = self.state
        out.active_track_id = int(self.active_track_id)
        out.remaining_estimate = int(self._remaining_viable_count())
        out.finished = bool(finished)
        out.note = self.note
        self.pub_state.publish(out)

    def _is_blacklisted(self, track_id):
        until = self.blacklist_until.get(track_id, 0.0)
        return time.time() < until

    @staticmethod
    def _pose_dist(a, b):
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        return math.hypot(dx, dy)

    def _robot_pose_map(self):
        p = Pose()
        p.orientation.w = 1.0
        p.position.x = self.robot_x
        p.position.y = self.robot_y
        p.position.z = 0.0
        return p

    def _to_map_pose(self, pose, src_frame, stamp):
        if src_frame == self.map_frame:
            return pose

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = src_frame
        ps.pose = pose

        try:
            out = self.tf_buffer.transform(ps, self.map_frame, timeout=rospy.Duration(0.05))
            return out.pose
        except Exception:
            return None

    def _validate_startup_data(self):
        if not self.validate_startup_data:
            return True, "startup validation disabled"

        deadline = time.time() + self.startup_validation_timeout_s
        while time.time() < deadline and not rospy.is_shutdown():
            if self.startup_homes_msg is not None:
                break
            rospy.sleep(0.05)

        homes_count = 0
        pucks_count = 0

        if self.startup_homes_msg is not None:
            homes_count = len({self._normalize_color(h.color) for h in self.startup_homes_msg.homes})
        if self.startup_pucks_msg is not None:
            pucks_count = len(self.startup_pucks_msg.tracks)

        ok = homes_count >= self.min_startup_homes
        return ok, f"startup homes={homes_count} pucks={pucks_count}"

    def _seed_homes_from_startup(self):
        if self.startup_homes_msg is None:
            return False

        seeded = HomeBaseArray()
        seeded.header.stamp = rospy.Time.now()
        seeded.header.frame_id = self.map_frame
        dedup = {}

        for h in self.startup_homes_msg.homes:
            pose_map = self._to_map_pose(h.pose_map, self.startup_homes_msg.header.frame_id, self.startup_homes_msg.header.stamp)
            if pose_map is None:
                continue

            color = self._normalize_color(h.color)
            if not color:
                continue

            out = HomeBase()
            out.header = seeded.header
            out.color = color
            out.pose_map = pose_map
            out.qr_payload = h.qr_payload
            out.marker_distance_m = h.marker_distance_m
            dedup[color] = out

        seeded.homes = list(dedup.values())

        if len({h.color for h in seeded.homes}) < self.min_startup_homes:
            return False

        self.homes = {h.color: h.pose_map for h in seeded.homes}
        self.pub_homes_seed.publish(seeded)
        return True

    def _path_len_from_plan(self, poses):
        if len(poses) < 2:
            return None

        total = 0.0
        for i in range(1, len(poses)):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            total += math.hypot(p1.x - p0.x, p1.y - p0.y)
        return total

    def _estimate_move_cost(self, start_pose, end_pose, cache):
        key = (
            round(start_pose.position.x, 2),
            round(start_pose.position.y, 2),
            round(end_pose.position.x, 2),
            round(end_pose.position.y, 2),
        )
        if key in cache:
            return cache[key]

        euc = self._pose_dist(start_pose, end_pose)
        cost = euc

        if self.use_make_plan_cost and self.make_plan_srv is not None:
            req = GetPlanRequest()

            req.start = PoseStamped()
            req.start.header.stamp = rospy.Time.now()
            req.start.header.frame_id = self.map_frame
            req.start.pose = start_pose

            req.goal = PoseStamped()
            req.goal.header.stamp = rospy.Time.now()
            req.goal.header.frame_id = self.map_frame
            req.goal.pose = end_pose

            req.tolerance = self.make_plan_tolerance

            try:
                rsp = self.make_plan_srv(req)
                plan_len = self._path_len_from_plan(rsp.plan.poses)
                if plan_len is not None:
                    cost = max(euc, plan_len)
                else:
                    cost = euc * 1.15
            except Exception:
                cost = euc

        cache[key] = cost
        return cost

    def _wall_penalty(self, pose):
        dist_to_wall = min(
            pose.position.x - self.rect_x_min,
            self.rect_x_max - pose.position.x,
            pose.position.y - self.rect_y_min,
            self.rect_y_max - pose.position.y,
        )

        if dist_to_wall >= self.wall_penalty_distance_m:
            return 0.0
        if dist_to_wall <= 0.0:
            return self.wall_penalty_weight * 2.0

        return self.wall_penalty_weight * (1.0 - dist_to_wall / max(1e-6, self.wall_penalty_distance_m))

    def _turn_penalty(self, pose):
        if not self.have_robot_pose:
            return 0.0
        bearing = math.atan2(pose.position.y - self.robot_y, pose.position.x - self.robot_x)
        err = abs(angle_wrap(bearing - self.robot_yaw))
        return self.turn_penalty_weight * (err / math.pi)

    def _candidate_penalty(self, track):
        conf_pen = self.confidence_penalty_weight * (1.0 - max(0.0, min(1.0, track.confidence)))
        attempts_pen = self.attempt_penalty_weight * float(self.attempt_counts.get(track.track_id, 0))
        wall_pen = self._wall_penalty(track.pose_map)
        turn_pen = self._turn_penalty(track.pose_map)
        return conf_pen + attempts_pen + wall_pen + turn_pen

    def _build_candidates(self):
        candidates = []
        for tr in self.tracks.values():
            if tr.state != "DETECTED":
                continue
            if tr.confidence < self.track_confidence_min:
                continue
            if self._is_blacklisted(tr.track_id):
                continue
            color = self._normalize_color(tr.color)
            if self.single_puck_per_color and color in self.completed_colors:
                continue
            if self.require_all_expected_colors and self.expected_colors and color not in self.expected_colors:
                continue
            home_pose = self.homes.get(color)
            if home_pose is None:
                continue

            candidates.append(
                {
                    "track": tr,
                    "home_pose": home_pose,
                    "color": color,
                }
            )
        return candidates

    def _select_best_target(self):
        candidates = self._build_candidates()
        if not candidates:
            return None

        robot_pose = self._robot_pose_map()
        cache = {}

        for c in candidates:
            tr = c["track"]
            home = c["home_pose"]
            to_pick = self._estimate_move_cost(robot_pose, tr.pose_map, cache)
            to_home = self._estimate_move_cost(tr.pose_map, home, cache)
            penalty = self._candidate_penalty(tr)
            c["primary"] = to_pick + to_home + penalty
            c["to_pick"] = to_pick
            c["to_home"] = to_home
            c["penalty"] = penalty

        if self.single_puck_per_color:
            best_by_color = {}
            for c in candidates:
                prev = best_by_color.get(c["color"])
                if prev is None or c["primary"] < prev["primary"]:
                    best_by_color[c["color"]] = c
            candidates = list(best_by_color.values())

        policy = str(self.target_policy).strip().lower()
        if policy in ("nearest", "nearest_reachable"):
            for c in candidates:
                c["score_total"] = c["to_pick"] + c["penalty"]
                c["lookahead"] = 0.0
            candidates.sort(key=lambda x: x["score_total"])
            return candidates[0]

        if policy == "highest_confidence":
            for c in candidates:
                c["score_total"] = -float(c["track"].confidence) + 0.1 * c["penalty"]
                c["lookahead"] = 0.0
            candidates.sort(key=lambda x: x["score_total"])
            return candidates[0]

        if policy not in ("cost_optimal", "cost", ""):
            rospy.logwarn_throttle(10.0, "unknown target_policy '%s'; using cost_optimal", self.target_policy)

        candidates.sort(key=lambda x: x["primary"])
        if self.lookahead_top_k > 0:
            pool = candidates[: min(len(candidates), self.lookahead_top_k)]
        else:
            pool = candidates

        best = None
        best_score = 1e18
        for c in pool:
            next_cost = 0.0
            others = [o for o in candidates if o["track"].track_id != c["track"].track_id]
            if others:
                best_next = 1e18
                for o in others:
                    move1 = self._estimate_move_cost(c["home_pose"], o["track"].pose_map, cache)
                    move2 = self._estimate_move_cost(o["track"].pose_map, o["home_pose"], cache)
                    nxt = move1 + move2 + 0.6 * o["penalty"]
                    if nxt < best_next:
                        best_next = nxt
                if best_next < 1e17:
                    next_cost = best_next

            total = c["primary"] + self.lookahead_weight * next_cost
            c["score_total"] = total
            c["lookahead"] = next_cost

            if total < best_score:
                best_score = total
                best = c

        return best

    def _reserve_specific_track(self, track_id):
        return self.reserve_srv(strategy=f"id:{int(track_id)}")

    def _handle_action_result(self):
        result = self.pick_client.get_result()
        if result is None:
            return

        if result.success:
            if self.single_puck_per_color and self.active_track_color:
                self.completed_colors.add(self.active_track_color)
            self.note = f"track {self.active_track_id} delivered ({self.active_track_color})"
        else:
            self.note = f"track {self.active_track_id} failed at {result.stage_failed}"
            self.attempt_counts[self.active_track_id] = self.attempt_counts.get(self.active_track_id, 0) + 1
            if self.attempt_counts[self.active_track_id] >= self.max_pick_attempts_per_track:
                self.blacklist_until[self.active_track_id] = time.time() + self.blacklist_duration_s
                self.note += " (temporarily blacklisted)"

        self.active_track_id = 0
        self.active_track_color = ""
        self.goal_in_flight = False

    def _done_condition(self):
        if self.single_puck_per_color and self.expected_colors:
            pending = self._pending_expected_colors()
            if not pending and not self.holding_object and not self.goal_in_flight:
                return True
            if pending:
                self.last_empty_start = None
                return False

        remaining = self._remaining_viable_count()

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

        startup_valid = False
        if self.startup_survey_required:
            self.state = "STARTUP_SURVEY"
            self.note = "running startup 360 survey"
            self._publish_state()
            try:
                rsp = self.startup_survey_srv()
                if not rsp.success and self.startup_survey_must_succeed:
                    self.state = "ERROR"
                    self.note = f"startup survey failed: {rsp.message}"
                    self._publish_state()
                    return
                self.note = f"startup survey: {rsp.message}"
                self._publish_state()
            except Exception as exc:
                if self.startup_survey_must_succeed:
                    self.state = "ERROR"
                    self.note = f"startup survey exception: {exc}"
                    self._publish_state()
                    return
                self.note = f"startup survey skipped (exception): {exc}"
                self._publish_state()

            startup_valid, validate_msg = self._validate_startup_data()
            self.state = "VALIDATE_STARTUP_DATA"
            self.note = validate_msg
            self._publish_state()

        if startup_valid:
            if self._seed_homes_from_startup():
                self.note = "startup homes seeded into /home_bases"
                self._publish_state()

        do_corner_scan = self.home_scan_required
        if do_corner_scan and startup_valid and self.skip_corner_scan_if_startup_valid and len(self.homes) >= self.min_startup_homes:
            do_corner_scan = False
            self.note = "skipping corner scan: startup data valid"
            self._publish_state()

        if do_corner_scan:
            self.state = "SCAN_HOMES"
            self.note = "scanning home markers"
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

            if self.holding_object:
                self.state = "WAITING_FOR_RELEASE"
                self.note = "gripper still reports a held puck; waiting for release confirmation"
                self._publish_state()
                rate.sleep()
                continue

            if self._done_condition():
                self.state = "FINISHED"
                if self.single_puck_per_color and self.expected_colors:
                    self.note = "all expected colors delivered"
                else:
                    self.note = "all pucks sorted"
                self._publish_state(finished=True)
                return

            selected = self._select_best_target()
            if selected is not None:
                self.no_target_since = None
                tr = selected["track"]
                try:
                    reserve_rsp = self._reserve_specific_track(tr.track_id)
                except Exception as exc:
                    self.note = f"reserve service error: {exc}"
                    self._publish_state()
                    rate.sleep()
                    continue

                if not reserve_rsp.success or reserve_rsp.track_id <= 0:
                    self.note = f"target {tr.track_id} unavailable; retrying selection"
                    self._publish_state()
                    rate.sleep()
                    continue

                if self._is_blacklisted(reserve_rsp.track_id):
                    self.pub_release.publish(UInt32(data=reserve_rsp.track_id))
                    self.note = f"reserved track {reserve_rsp.track_id} is blacklisted"
                    self._publish_state()
                    rate.sleep()
                    continue

                reserve_color = self._normalize_color(reserve_rsp.color)
                if self.single_puck_per_color and reserve_color in self.completed_colors:
                    self.pub_release.publish(UInt32(data=reserve_rsp.track_id))
                    self.note = f"color {reserve_color} already delivered"
                    self._publish_state()
                    rate.sleep()
                    continue

                if self.require_all_expected_colors and self.expected_colors and reserve_color not in self.expected_colors:
                    self.pub_release.publish(UInt32(data=reserve_rsp.track_id))
                    self.note = f"reserved color {reserve_color} not in expected_colors"
                    self._publish_state()
                    rate.sleep()
                    continue

                home_pose = self.homes.get(reserve_color)
                if home_pose is None:
                    self.pub_release.publish(UInt32(data=reserve_rsp.track_id))
                    self.note = f"home for color {reserve_color} unavailable"
                    self._publish_state()
                    rate.sleep()
                    continue

                goal = PickAndPlaceGoal()
                goal.track_id = reserve_rsp.track_id
                goal.color = reserve_color
                goal.pick_pose = reserve_rsp.pose_map
                goal.place_pose = home_pose

                self.pick_client.send_goal(goal)
                self.active_track_id = reserve_rsp.track_id
                self.active_track_color = reserve_color
                self.goal_in_flight = True

                self.note = (
                    f"pick track {self.active_track_id} score={selected.get('score_total', 0.0):.2f} "
                    f"(to_pick={selected.get('to_pick', 0.0):.2f}, to_home={selected.get('to_home', 0.0):.2f})"
                )
                self._publish_state()
                rate.sleep()
                continue

            if self.no_target_since is None:
                self.no_target_since = rospy.Time.now()

            idle_for = (rospy.Time.now() - self.no_target_since).to_sec()
            if idle_for < self.search_trigger_s:
                self.state = "RUNNING"
                if self.single_puck_per_color and self.expected_colors:
                    pending = ",".join(self._pending_expected_colors())
                    self.note = (
                        f"no viable target; pending colors [{pending}] "
                        f"waiting {idle_for:.1f}/{self.search_trigger_s:.1f}s"
                    )
                else:
                    self.note = f"no viable target; waiting {idle_for:.1f}/{self.search_trigger_s:.1f}s"
                self._publish_state()
                rate.sleep()
                continue

            self.state = "SEARCH"
            if self.single_puck_per_color and self.expected_colors:
                pending = ",".join(self._pending_expected_colors())
                self.note = f"no viable target; pending [{pending}]; running coverage search"
            else:
                self.note = "no viable target; running coverage search"
            self._publish_state()
            try:
                rsp = self.coverage_srv()
                self.note = rsp.message
            except Exception as exc:
                self.note = f"coverage service error: {exc}"

            self.no_target_since = None
            self.state = "RUNNING"
            self._publish_state()
            rate.sleep()


def main():
    rospy.init_node("mission_manager")
    manager = MissionManager()
    manager.run()


if __name__ == "__main__":
    main()
