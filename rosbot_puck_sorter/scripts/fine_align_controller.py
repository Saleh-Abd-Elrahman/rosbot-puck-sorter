#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

from rosbot_puck_sorter.msg import PuckDetectionArray, PuckTrack


class FineAlignController:
    def __init__(self):
        self.lock = threading.Lock()

        self.control_rate_hz = rospy.get_param("~control_rate_hz", 20.0)
        self.kp_yaw = rospy.get_param("~kp_yaw", 0.005)
        self.kp_fwd = rospy.get_param("~kp_fwd", 0.0025)
        self.max_ang_vel = rospy.get_param("~max_ang_vel_rad_s", 0.7)
        self.max_lin_vel = rospy.get_param("~max_lin_vel_m_s", 0.12)
        self.yaw_deadband_m = rospy.get_param("~yaw_deadband_m", rospy.get_param("~yaw_deadband_px", 0.02))
        self.depth_deadband_m = rospy.get_param("~depth_deadband_m", 0.015)
        self.target_depth_m = rospy.get_param("~target_depth_m", 0.18)
        self.success_hold_s = rospy.get_param("~success_hold_s", 0.4)
        self.align_timeout_s = rospy.get_param("~align_timeout_s", 6.0)

        self.current_target_color = ""
        self.latest_detections = []

        self.pub_cmd = rospy.Publisher("/cmd_vel_align", Twist, queue_size=10)
        rospy.Subscriber("/puck/detections", PuckDetectionArray, self._detections_cb, queue_size=10)
        rospy.Subscriber("/mission/current_target", PuckTrack, self._target_cb, queue_size=10)

        rospy.Service("/fine_align/execute", Trigger, self._execute_cb)
        rospy.loginfo("fine_align_controller ready")

    def _detections_cb(self, msg):
        with self.lock:
            self.latest_detections = list(msg.detections)

    def _target_cb(self, msg):
        with self.lock:
            self.current_target_color = msg.color

    def _select_detection(self):
        with self.lock:
            detections = list(self.latest_detections)
            target_color = self.current_target_color

        if not detections:
            return None

        if target_color:
            color_dets = [d for d in detections if d.color == target_color]
            if color_dets:
                return min(color_dets, key=lambda d: d.position_camera.z)

        return min(detections, key=lambda d: d.position_camera.z)

    def _stop(self):
        self.pub_cmd.publish(Twist())

    def _execute_cb(self, _req):
        rate = rospy.Rate(self.control_rate_hz)
        start = rospy.Time.now()
        stable_start = None

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > self.align_timeout_s:
                self._stop()
                return TriggerResponse(success=False, message="align timeout")

            det = self._select_detection()
            if det is None:
                self._stop()
                rate.sleep()
                continue

            lateral_err = det.position_camera.x
            depth_err = det.position_camera.z - self.target_depth_m

            aligned = abs(lateral_err) <= self.yaw_deadband_m and abs(depth_err) <= self.depth_deadband_m
            if aligned:
                if stable_start is None:
                    stable_start = rospy.Time.now()
                if (rospy.Time.now() - stable_start).to_sec() >= self.success_hold_s:
                    self._stop()
                    return TriggerResponse(success=True, message="aligned")
            else:
                stable_start = None

            cmd = Twist()
            cmd.angular.z = max(-self.max_ang_vel, min(self.max_ang_vel, -self.kp_yaw * lateral_err))
            cmd.linear.x = max(-self.max_lin_vel, min(self.max_lin_vel, self.kp_fwd * depth_err))

            # Avoid backing up in close-range fine alignment.
            if cmd.linear.x < 0.0:
                cmd.linear.x = 0.0

            self.pub_cmd.publish(cmd)
            rate.sleep()

        self._stop()
        return TriggerResponse(success=False, message="shutdown")


def main():
    rospy.init_node("fine_align_controller")
    FineAlignController()
    rospy.spin()


if __name__ == "__main__":
    main()
