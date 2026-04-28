#!/usr/bin/env python3
import math
import threading
from dataclasses import dataclass

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, UInt32
from rosbot_puck_sorter.msg import (
    HomeBaseArray,
    PuckDetectionArray,
    PuckTrack,
    PuckTrackArray,
)
from rosbot_puck_sorter.srv import ReserveTarget, ReserveTargetResponse


@dataclass
class TrackState:
    track_id: int
    color: str
    pose_map: Pose
    confidence: float
    state: str
    miss_count: int
    last_seen: rospy.Time
    hits: int
    reserved_at: rospy.Time


class PuckWorldModel:
    def __init__(self):
        self.lock = threading.Lock()

        self.update_rate_hz = rospy.get_param("~update_rate_hz", 15.0)
        self.association_max_dist_m = rospy.get_param("~association_max_dist_m", 0.18)
        self.association_max_dt_s = rospy.get_param("~association_max_dt_s", 1.0)
        self.confirm_hits_required = int(rospy.get_param("~confirm_hits_required", 3))
        self.misses_to_lost = int(rospy.get_param("~misses_to_lost", 8))
        self.track_ttl_s = rospy.get_param("~track_ttl_s", 10.0)
        self.conf_init = rospy.get_param("~conf_init", 0.40)
        self.conf_hit_gain = rospy.get_param("~conf_hit_gain", 0.12)
        self.conf_miss_decay = rospy.get_param("~conf_miss_decay", 0.08)
        self.conf_max = rospy.get_param("~conf_max", 0.99)
        self.conf_valid_min = rospy.get_param("~conf_valid_min", 0.55)
        self.merge_dist_m = rospy.get_param("~merge_dist_m", 0.10)
        self.reserve_timeout_s = rospy.get_param("~reserve_timeout_s", 30.0)
        self.home_exclusion_radius_m = rospy.get_param("~home_exclusion_radius_m", 0.22)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.start_frame = rospy.get_param("~start_frame", "start")
        self.publish_start_relative = bool(rospy.get_param("~publish_start_relative", True))
        self.pose_source = str(rospy.get_param("~pose_source", "auto")).strip().lower()
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.tracks = {}
        self.next_track_id = 1
        self.homes = {}
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.have_robot_pose = False

        self.pub_tracks = rospy.Publisher("/puck/tracks", PuckTrackArray, queue_size=10)
        self.pub_tracks_start = rospy.Publisher("/puck/tracks_start", PuckTrackArray, queue_size=10)
        self.pub_remaining = rospy.Publisher("/puck/remaining_count", UInt32, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/puck/detections", PuckDetectionArray, self._detections_cb, queue_size=10)
        rospy.Subscriber("/home_bases", HomeBaseArray, self._homes_cb, queue_size=1)
        if self.pose_source in ("auto", "amcl", "pose", "pose_topic"):
            rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self._amcl_pose_cb, queue_size=10)
        if self.pose_source in ("auto", "odom"):
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber("/puck_world_model/delivered_track", UInt32, self._delivered_cb, queue_size=10)
        rospy.Subscriber("/puck_world_model/release_track", UInt32, self._release_cb, queue_size=10)
        rospy.Subscriber("/puck_world_model/lost_track", UInt32, self._lost_cb, queue_size=10)

        self.srv_reserve = rospy.Service("/puck_world_model/reserve_target", ReserveTarget, self._reserve_target)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-3, self.update_rate_hz)), self._timer_cb)

        rospy.loginfo("puck_world_model ready")

    def _amcl_pose_cb(self, msg):
        self._set_robot_pose(msg.pose.pose)

    def _odom_cb(self, msg):
        self._set_robot_pose(msg.pose.pose)

    def _set_robot_pose(self, pose):
        with self.lock:
            self.robot_x = pose.position.x
            self.robot_y = pose.position.y
            self.have_robot_pose = True

    def _homes_cb(self, msg):
        with self.lock:
            self.homes = {}
            for home in msg.homes:
                self.homes[home.color] = home.pose_map

    def _distance_pose(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return math.hypot(dx, dy)

    def _near_any_home(self, pose):
        for home in self.homes.values():
            dx = pose.position.x - home.position.x
            dy = pose.position.y - home.position.y
            if math.hypot(dx, dy) <= self.home_exclusion_radius_m:
                return True
        return False

    def _make_pose(self, point):
        pose = Pose()
        pose.position.x = point.x
        pose.position.y = point.y
        pose.position.z = point.z
        pose.orientation.w = 1.0
        return pose

    def _to_start_pose(self, pose_map, stamp):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.map_frame
        ps.pose = pose_map
        try:
            out = self.tf_buffer.transform(ps, self.start_frame, timeout=rospy.Duration(0.02))
            return out.pose
        except Exception:
            return None

    def _associate(self, color, pose):
        now = rospy.Time.now()
        best_id = None
        best_dist = 1e9
        for tid, tr in self.tracks.items():
            if tr.color != color:
                continue
            if tr.state in ("DELIVERED", "LOST"):
                continue
            if self.association_max_dt_s > 0.0 and (now - tr.last_seen).to_sec() > self.association_max_dt_s:
                continue
            dist = self._distance_pose(tr.pose_map, pose)
            if dist < self.association_max_dist_m and dist < best_dist:
                best_id = tid
                best_dist = dist
        return best_id

    def _detections_cb(self, msg):
        with self.lock:
            now = rospy.Time.now()
            touched = set()
            for det in msg.detections:
                pose = self._make_pose(det.position_map)
                if self._near_any_home(pose):
                    continue

                tid = self._associate(det.color, pose)
                if tid is None:
                    tid = self.next_track_id
                    self.next_track_id += 1
                    self.tracks[tid] = TrackState(
                        track_id=tid,
                        color=det.color,
                        pose_map=pose,
                        confidence=float(self.conf_init),
                        state="DETECTED" if self.confirm_hits_required <= 1 else "CANDIDATE",
                        miss_count=0,
                        last_seen=now,
                        hits=1,
                        reserved_at=rospy.Time(0),
                    )
                else:
                    tr = self.tracks[tid]
                    alpha = 0.4
                    tr.pose_map.position.x = alpha * pose.position.x + (1.0 - alpha) * tr.pose_map.position.x
                    tr.pose_map.position.y = alpha * pose.position.y + (1.0 - alpha) * tr.pose_map.position.y
                    tr.pose_map.position.z = alpha * pose.position.z + (1.0 - alpha) * tr.pose_map.position.z
                    tr.confidence = min(self.conf_max, tr.confidence + self.conf_hit_gain)
                    tr.miss_count = 0
                    tr.last_seen = now
                    tr.hits += 1
                    if tr.state == "CANDIDATE" and tr.hits >= self.confirm_hits_required:
                        tr.state = "DETECTED"
                    if tr.state == "LOST":
                        tr.state = "DETECTED"
                    if tr.state == "RESERVED" and (now - tr.reserved_at).to_sec() > self.reserve_timeout_s:
                        tr.state = "DETECTED"

                touched.add(tid)

            for tid, tr in self.tracks.items():
                if tid in touched:
                    continue
                if tr.state in ("DELIVERED", "LOST"):
                    continue
                if tr.state == "RESERVED":
                    if (now - tr.reserved_at).to_sec() > self.reserve_timeout_s:
                        tr.state = "DETECTED"
                        tr.miss_count = 0
                        tr.last_seen = now
                    continue
                tr.miss_count += 1
                tr.confidence = max(0.0, tr.confidence - self.conf_miss_decay)
                if tr.miss_count >= self.misses_to_lost:
                    tr.state = "LOST"

    def _delivered_cb(self, msg):
        with self.lock:
            tr = self.tracks.get(msg.data)
            if tr:
                tr.state = "DELIVERED"
                tr.confidence = 0.0

    def _release_cb(self, msg):
        with self.lock:
            tr = self.tracks.get(msg.data)
            if tr and tr.state == "RESERVED":
                tr.state = "DETECTED"
                tr.miss_count = 0
                tr.last_seen = rospy.Time.now()
                tr.confidence = max(tr.confidence, self.conf_valid_min)

    def _lost_cb(self, msg):
        with self.lock:
            tr = self.tracks.get(msg.data)
            if tr and tr.state != "DELIVERED":
                tr.state = "LOST"
                tr.confidence = 0.0
                tr.miss_count = self.misses_to_lost
                tr.last_seen = rospy.Time.now()

    def _select_candidate(self, strategy):
        if isinstance(strategy, str) and strategy.startswith("id:"):
            try:
                req_id = int(strategy.split(":", 1)[1])
            except Exception:
                req_id = -1
            tr = self.tracks.get(req_id)
            if tr is None:
                return None
            if tr.state != "DETECTED":
                return None
            if tr.confidence < self.conf_valid_min:
                return None
            return tr

        candidates = []
        for tr in self.tracks.values():
            if tr.state != "DETECTED":
                continue
            if tr.confidence < self.conf_valid_min:
                continue
            candidates.append(tr)

        if not candidates:
            return None

        if strategy == "highest_confidence":
            candidates.sort(key=lambda t: t.confidence, reverse=True)
        else:
            if self.have_robot_pose:
                candidates.sort(
                    key=lambda t: math.hypot(t.pose_map.position.x - self.robot_x, t.pose_map.position.y - self.robot_y)
                )
            else:
                candidates.sort(key=lambda t: math.hypot(t.pose_map.position.x, t.pose_map.position.y))

        return candidates[0]

    def _reserve_target(self, req):
        with self.lock:
            tr = self._select_candidate(req.strategy)
            if tr is None:
                return ReserveTargetResponse(
                    success=False,
                    track_id=0,
                    color="",
                    pose_map=Pose(),
                    message="no target available",
                )

            tr.state = "RESERVED"
            tr.reserved_at = rospy.Time.now()
            tr.miss_count = 0
            return ReserveTargetResponse(
                success=True,
                track_id=tr.track_id,
                color=tr.color,
                pose_map=tr.pose_map,
                message="reserved",
            )

    def _timer_cb(self, _event):
        with self.lock:
            now = rospy.Time.now()
            for tr in self.tracks.values():
                if tr.state == "RESERVED" and (now - tr.reserved_at).to_sec() > self.reserve_timeout_s:
                    tr.state = "DETECTED"
                    tr.miss_count = 0
                    tr.last_seen = now
                if tr.state in ("DELIVERED",):
                    continue
                if (now - tr.last_seen).to_sec() > self.track_ttl_s and tr.state != "RESERVED":
                    tr.state = "LOST"

            # Merge duplicates conservatively.
            ids = sorted(self.tracks.keys())
            removed = set()
            for i in range(len(ids)):
                a_id = ids[i]
                if a_id in removed or a_id not in self.tracks:
                    continue
                a = self.tracks[a_id]
                if a.state in ("DELIVERED", "LOST"):
                    continue
                for j in range(i + 1, len(ids)):
                    b_id = ids[j]
                    if b_id in removed or b_id not in self.tracks:
                        continue
                    b = self.tracks[b_id]
                    if b.color != a.color:
                        continue
                    if b.state in ("DELIVERED", "LOST"):
                        continue
                    if self._distance_pose(a.pose_map, b.pose_map) <= self.merge_dist_m:
                        if a.state == "RESERVED" and b.state != "RESERVED":
                            keep, drop = a_id, b_id
                        elif b.state == "RESERVED" and a.state != "RESERVED":
                            keep, drop = b_id, a_id
                        else:
                            keep, drop = (a_id, b_id) if a.confidence >= b.confidence else (b_id, a_id)
                        removed.add(drop)
                        self.tracks[keep].confidence = max(self.tracks[keep].confidence, self.tracks[drop].confidence)

            for rid in removed:
                self.tracks.pop(rid, None)

            msg = PuckTrackArray()
            msg.header = Header(stamp=now, frame_id=self.map_frame)
            msg_start = PuckTrackArray()
            msg_start.header = Header(stamp=now, frame_id=self.start_frame)
            remaining = 0
            for tr in sorted(self.tracks.values(), key=lambda t: t.track_id):
                out = PuckTrack()
                out.header = msg.header
                out.track_id = tr.track_id
                out.color = tr.color
                out.pose_map = tr.pose_map
                out.confidence = tr.confidence
                out.state = tr.state
                out.miss_count = tr.miss_count
                out.last_seen = tr.last_seen
                msg.tracks.append(out)
                if tr.state == "DETECTED" and tr.confidence >= self.conf_valid_min:
                    remaining += 1

                if self.publish_start_relative:
                    pose_start = self._to_start_pose(tr.pose_map, now)
                    if pose_start is not None:
                        out_start = PuckTrack()
                        out_start.header = msg_start.header
                        out_start.track_id = tr.track_id
                        out_start.color = tr.color
                        out_start.pose_map = pose_start
                        out_start.confidence = tr.confidence
                        out_start.state = tr.state
                        out_start.miss_count = tr.miss_count
                        out_start.last_seen = tr.last_seen
                        msg_start.tracks.append(out_start)

            self.pub_tracks.publish(msg)
            if self.publish_start_relative:
                self.pub_tracks_start.publish(msg_start)
            self.pub_remaining.publish(UInt32(data=remaining))


def main():
    rospy.init_node("puck_world_model")
    PuckWorldModel()
    rospy.spin()


if __name__ == "__main__":
    main()
