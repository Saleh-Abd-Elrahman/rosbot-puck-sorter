#!/usr/bin/env python3
import threading

import rospy
from std_msgs.msg import Bool, Float32

from rosbot_puck_sorter.srv import SetGripper, SetGripperResponse


class ServoBackend:
    def __init__(self, channel, frequency_hz):
        self.channel = channel
        self.frequency_hz = frequency_hz
        self.mode = "mock"
        self.pi = None

        try:
            import pigpio

            self.pigpio = pigpio
            self.pi = pigpio.pi()
            if self.pi.connected:
                self.pi.set_PWM_frequency(self.channel, self.frequency_hz)
                self.mode = "pigpio"
            else:
                self.pi = None
                rospy.logwarn("pigpio not connected; running mock gripper backend")
        except Exception:
            rospy.logwarn("pigpio unavailable; running mock gripper backend")

    def set_pulse(self, pulse_us):
        if self.mode == "pigpio" and self.pi is not None:
            self.pi.set_servo_pulsewidth(self.channel, int(pulse_us))

    def shutdown(self):
        if self.mode == "pigpio" and self.pi is not None:
            self.pi.set_servo_pulsewidth(self.channel, 0)
            self.pi.stop()


class GripperController:
    def __init__(self):
        self.lock = threading.Lock()

        self.pwm_channel = int(rospy.get_param("~pwm_channel", 0))
        self.pwm_frequency_hz = int(rospy.get_param("~pwm_frequency_hz", 50))
        self.min_pulse_us = int(rospy.get_param("~min_pulse_us", 500))
        self.max_pulse_us = int(rospy.get_param("~max_pulse_us", 2500))
        self.open_angle_deg = float(rospy.get_param("~open_angle_deg", 20.0))
        self.close_angle_deg = float(rospy.get_param("~close_angle_deg", 95.0))
        self.settle_time_s = float(rospy.get_param("~settle_time_s", 0.4))
        self.invert_direction = bool(rospy.get_param("~invert_direction", False))
        self.open_on_shutdown = bool(rospy.get_param("~open_on_shutdown", True))

        self.backend = ServoBackend(self.pwm_channel, self.pwm_frequency_hz)

        self.current_angle = self.open_angle_deg
        self.holding_object = False

        self.pub_angle = rospy.Publisher("/gripper/state", Float32, queue_size=10, latch=True)
        self.pub_hold = rospy.Publisher("/gripper/holding_object", Bool, queue_size=10, latch=True)

        self.service = rospy.Service("/gripper/set", SetGripper, self._set_cb)

        self._send_angle(self.current_angle)
        self._publish_state()

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("gripper_controller ready (backend=%s)", self.backend.mode)

    def _angle_to_pulse(self, angle_deg):
        # Map [0,180] deg to configured pulse window.
        angle_deg = max(0.0, min(180.0, angle_deg))
        ratio = angle_deg / 180.0
        pulse = self.min_pulse_us + ratio * (self.max_pulse_us - self.min_pulse_us)
        return pulse

    def _send_angle(self, angle_deg):
        if self.invert_direction:
            angle_deg = 180.0 - angle_deg
        pulse = self._angle_to_pulse(angle_deg)
        self.backend.set_pulse(pulse)
        self.current_angle = angle_deg

    def _publish_state(self):
        self.pub_angle.publish(Float32(data=float(self.current_angle)))
        self.pub_hold.publish(Bool(data=bool(self.holding_object)))

    def _set_cb(self, req):
        cmd = (req.command or "").strip().lower()
        with self.lock:
            if cmd == "open":
                self._send_angle(self.open_angle_deg)
                self.holding_object = False
            elif cmd == "close":
                self._send_angle(self.close_angle_deg)
                # Without force sensing, close implies probable object hold.
                self.holding_object = True
            elif cmd == "angle":
                self._send_angle(float(req.angle_deg))
            else:
                return SetGripperResponse(success=False, message="command must be open|close|angle")

            rospy.sleep(self.settle_time_s)
            self._publish_state()
            return SetGripperResponse(success=True, message=f"gripper set via {cmd}")

    def _on_shutdown(self):
        if self.open_on_shutdown:
            try:
                self._send_angle(self.open_angle_deg)
            except Exception:
                pass
        self.backend.shutdown()


def main():
    rospy.init_node("gripper_controller")
    GripperController()
    rospy.spin()


if __name__ == "__main__":
    main()
