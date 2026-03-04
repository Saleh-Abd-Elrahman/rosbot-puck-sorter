#!/usr/bin/env python3
import math
import threading

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger, TriggerResponse

from rosbot_puck_sorter.msg import HomeBase, HomeBaseArray, PuckTrack, PuckTrackArray


class SemanticGridMapper:
    def __init__(self):
        self.lock = threading.Lock()

        self.start_frame = rospy.get_param("~start_frame", "start")
        self.grid_topic = rospy.get_param("~grid_topic", "/semantic_map/grid")
        self.homes_topic = rospy.get_param("~homes_topic", "/semantic_map/homes")
        self.pucks_topic = rospy.get_param("~pucks_topic", "/semantic_map/pucks")

        self.startup_homes_topic = rospy.get_param("~startup_homes_topic", "/startup_survey/homes_start")
        self.startup_pucks_topic = rospy.get_param("~startup_pucks_topic", "/startup_survey/pucks_start")
        self.dynamic_pucks_topic = rospy.get_param("~dynamic_pucks_topic", "/puck/tracks_start")

        self.resolution = float(rospy.get_param("~resolution", 0.05))
        self.rect_x_min = float(rospy.get_param("~rect_x_min", -0.5))
        self.rect_x_max = float(rospy.get_param("~rect_x_max", 5.5))
        self.rect_y_min = float(rospy.get_param("~rect_y_min", -0.5))
        self.rect_y_max = float(rospy.get_param("~rect_y_max", 4.5))

        self.wall_thickness_m = float(rospy.get_param("~wall_thickness_m", 0.12))
        self.home_radius_m = float(rospy.get_param("~home_radius_m", 0.12))
        self.puck_radius_m = float(rospy.get_param("~puck_radius_m", 0.06))

        self.wall_value = int(rospy.get_param("~wall_value", 100))
        self.home_value = int(rospy.get_param("~home_value", 100))
        self.puck_value = int(rospy.get_param("~puck_value", 80))

        self.use_dynamic_puck_tracks = bool(rospy.get_param("~use_dynamic_puck_tracks", True))
        self.allowed_puck_states = set(rospy.get_param("~allowed_puck_states", ["DETECTED", "RESERVED", "PICKED"]))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 1.0))

        self.homes_msg = None
        self.pucks_msg = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_grid = rospy.Publisher(self.grid_topic, OccupancyGrid, queue_size=1, latch=True)
        self.pub_homes = rospy.Publisher(self.homes_topic, HomeBaseArray, queue_size=1, latch=True)
        self.pub_pucks = rospy.Publisher(self.pucks_topic, PuckTrackArray, queue_size=1, latch=True)

        rospy.Subscriber(self.startup_homes_topic, HomeBaseArray, self._homes_cb, queue_size=1)
        rospy.Subscriber(self.startup_pucks_topic, PuckTrackArray, self._startup_pucks_cb, queue_size=1)
        rospy.Subscriber(self.dynamic_pucks_topic, PuckTrackArray, self._dynamic_pucks_cb, queue_size=10)

        self.srv_rebuild = rospy.Service("/semantic_map/rebuild", Trigger, self._rebuild_cb)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-3, self.publish_rate_hz)), self._timer_cb)

        rospy.loginfo("semantic_grid_mapper ready")

    def _to_start_pose(self, pose, src_frame, stamp):
        if src_frame == self.start_frame:
            return pose

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = src_frame
        ps.pose = pose
        try:
            out = self.tf_buffer.transform(ps, self.start_frame, timeout=rospy.Duration(0.03))
            return out.pose
        except Exception:
            return None

    def _grid_shape(self):
        width = int(math.ceil((self.rect_x_max - self.rect_x_min) / self.resolution))
        height = int(math.ceil((self.rect_y_max - self.rect_y_min) / self.resolution))
        width = max(1, width)
        height = max(1, height)
        return width, height

    def _xy_to_index(self, x, y, width, height):
        ix = int(math.floor((x - self.rect_x_min) / self.resolution))
        iy = int(math.floor((y - self.rect_y_min) / self.resolution))
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            return None
        return ix, iy

    @staticmethod
    def _set_cell(data, width, height, ix, iy, value):
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            return
        idx = iy * width + ix
        data[idx] = max(data[idx], value)

    def _paint_circle(self, data, width, height, x, y, radius_m, value):
        c = self._xy_to_index(x, y, width, height)
        if c is None:
            return
        cx, cy = c
        rc = max(1, int(math.ceil(radius_m / self.resolution)))
        r2 = rc * rc
        for dy in range(-rc, rc + 1):
            for dx in range(-rc, rc + 1):
                if (dx * dx + dy * dy) <= r2:
                    self._set_cell(data, width, height, cx + dx, cy + dy, value)

    def _build_grid(self):
        width, height = self._grid_shape()
        data = [0] * (width * height)

        wall_cells = max(1, int(math.ceil(self.wall_thickness_m / self.resolution)))

        for iy in range(height):
            for ix in range(width):
                if ix < wall_cells or ix >= (width - wall_cells) or iy < wall_cells or iy >= (height - wall_cells):
                    self._set_cell(data, width, height, ix, iy, self.wall_value)

        if self.homes_msg is not None:
            for h in self.homes_msg.homes:
                self._paint_circle(data, width, height, h.pose_map.position.x, h.pose_map.position.y, self.home_radius_m, self.home_value)

        if self.pucks_msg is not None:
            for p in self.pucks_msg.tracks:
                if p.state not in self.allowed_puck_states:
                    continue
                self._paint_circle(data, width, height, p.pose_map.position.x, p.pose_map.position.y, self.puck_radius_m, self.puck_value)

        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.start_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = self.rect_x_min
        msg.info.origin.position.y = self.rect_y_min
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def _publish_all_locked(self):
        if self.homes_msg is not None:
            self.pub_homes.publish(self.homes_msg)
        if self.pucks_msg is not None:
            self.pub_pucks.publish(self.pucks_msg)

        grid = self._build_grid()
        self.pub_grid.publish(grid)

    def _homes_cb(self, msg):
        with self.lock:
            transformed = HomeBaseArray()
            transformed.header.stamp = rospy.Time.now()
            transformed.header.frame_id = self.start_frame
            for h in msg.homes:
                pose = self._to_start_pose(h.pose_map, msg.header.frame_id, msg.header.stamp)
                if pose is None:
                    continue
                out = HomeBase()
                out.header = transformed.header
                out.color = h.color
                out.pose_map = pose
                out.qr_payload = h.qr_payload
                out.marker_distance_m = h.marker_distance_m
                transformed.homes.append(out)
            self.homes_msg = transformed

            self._publish_all_locked()

    def _startup_pucks_cb(self, msg):
        with self.lock:
            out = PuckTrackArray()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.start_frame

            for p in msg.tracks:
                if p.state not in self.allowed_puck_states:
                    continue

                pose = self._to_start_pose(p.pose_map, msg.header.frame_id, msg.header.stamp)
                if pose is None:
                    continue

                pp = PuckTrack()
                pp.header = out.header
                pp.track_id = p.track_id
                pp.color = p.color
                pp.pose_map = pose
                pp.confidence = p.confidence
                pp.state = p.state
                pp.miss_count = p.miss_count
                pp.last_seen = p.last_seen
                out.tracks.append(pp)

            self.pucks_msg = out
            self._publish_all_locked()

    def _dynamic_pucks_cb(self, msg):
        if not self.use_dynamic_puck_tracks:
            return

        with self.lock:
            out = PuckTrackArray()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.start_frame

            for p in msg.tracks:
                if p.state not in self.allowed_puck_states:
                    continue

                pp = PuckTrack()
                pp.header = out.header
                pp.track_id = p.track_id
                pp.color = p.color
                pp.confidence = p.confidence
                pp.state = p.state
                pp.miss_count = p.miss_count
                pp.last_seen = p.last_seen

                pose = self._to_start_pose(p.pose_map, msg.header.frame_id, msg.header.stamp)
                if pose is None:
                    continue
                pp.pose_map = pose

                out.tracks.append(pp)

            self.pucks_msg = out
            self._publish_all_locked()

    def _rebuild_cb(self, _req):
        with self.lock:
            self._publish_all_locked()
        return TriggerResponse(success=True, message="semantic map rebuilt")

    def _timer_cb(self, _event):
        with self.lock:
            self._publish_all_locked()


def main():
    rospy.init_node("semantic_grid_mapper")
    SemanticGridMapper()
    rospy.spin()


if __name__ == "__main__":
    main()
