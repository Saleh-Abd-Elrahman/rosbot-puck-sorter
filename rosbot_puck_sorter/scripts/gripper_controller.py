#!/usr/bin/env python3
import threading
import time

import rospy
from std_msgs.msg import Bool, Float32

from rosbot_puck_sorter.srv import SetGripper, SetGripperResponse


class MockBackend:
    def __init__(self):
        self.mode = "mock"
        self.last_command = ""

    def command(self, line, timeout_s):
        self.last_command = line.strip()
        return True, f"MOCK {self.last_command}"

    def shutdown(self):
        return


class PigpioBackend:
    def __init__(self, channel, frequency_hz):
        self.channel = int(channel)
        self.frequency_hz = int(frequency_hz)
        self.mode = "pigpio"
        self.pi = None

        try:
            import pigpio

            self.pigpio = pigpio
            self.pi = pigpio.pi()
            if self.pi.connected:
                self.pi.set_PWM_frequency(self.channel, self.frequency_hz)
            else:
                self.pi = None
                raise RuntimeError("pigpio daemon not connected")
        except Exception as exc:
            raise RuntimeError(f"pigpio backend unavailable: {exc}")

    def set_pulse(self, pulse_us):
        if self.pi is None:
            return False, "pigpio not connected"
        self.pi.set_servo_pulsewidth(self.channel, int(pulse_us))
        return True, f"OK PULSE {int(pulse_us)}"

    def command(self, line, timeout_s):
        parts = line.strip().split()
        if len(parts) != 2 or parts[0] != "PULSE_US":
            return False, f"ERR unsupported pigpio command: {line.strip()}"
        try:
            pulse_us = int(float(parts[1]))
        except Exception:
            return False, f"ERR invalid pulse value: {parts[1]}"
        return self.set_pulse(pulse_us)

    def shutdown(self):
        if self.pi is not None:
            self.pi.set_servo_pulsewidth(self.channel, 0)
            self.pi.stop()


class SerialBackend:
    def __init__(self, port, baud_rate, serial_timeout_s, startup_delay_s):
        self.mode = "arduino_serial"
        self.port = port
        self.baud_rate = int(baud_rate)
        self.serial_timeout_s = float(serial_timeout_s)
        self.startup_delay_s = float(startup_delay_s)
        self.ser = None

        try:
            import serial

            self.serial = serial
            factory = getattr(serial, "serial_for_url", serial.Serial)
            self.ser = factory(
                self.port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout_s,
                write_timeout=self.serial_timeout_s,
            )
            time.sleep(self.startup_delay_s)
            self._flush_buffers()
        except Exception as exc:
            raise RuntimeError(f"serial backend unavailable on {self.port}: {exc}")

    def _flush_buffers(self):
        if self.ser is None:
            return
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        try:
            self.ser.reset_output_buffer()
        except Exception:
            pass

    def command(self, line, timeout_s):
        if self.ser is None:
            return False, "serial port not open"

        wire = (line.strip() + "\n").encode("ascii", errors="ignore")
        deadline = time.time() + max(0.05, float(timeout_s))

        try:
            self.ser.write(wire)
            self.ser.flush()
        except Exception as exc:
            return False, f"serial write failed: {exc}"

        response = ""
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                return False, f"serial read failed: {exc}"

            if not raw:
                continue

            response = raw.decode("ascii", errors="replace").strip()
            if not response:
                continue
            if response.upper().startswith("OK"):
                return True, response
            if response.upper().startswith("ERR"):
                return False, response

        return False, f"timeout waiting for Arduino reply to '{line.strip()}'"

    def shutdown(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


class GripperController:
    def __init__(self):
        self.lock = threading.Lock()

        self.backend_name = str(rospy.get_param("~backend", rospy.get_param("~pwm_driver", "arduino_serial"))).strip().lower()

        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.serial_baud_rate = int(rospy.get_param("~serial_baud_rate", 115200))
        self.serial_timeout_s = float(rospy.get_param("~serial_timeout_s", 0.3))
        self.command_timeout_s = float(rospy.get_param("~command_timeout_s", 1.0))
        self.startup_delay_s = float(rospy.get_param("~startup_delay_s", 2.0))

        self.pwm_channel = int(rospy.get_param("~pwm_channel", 0))
        self.pwm_frequency_hz = int(rospy.get_param("~pwm_frequency_hz", 50))
        self.min_pulse_us = int(rospy.get_param("~min_pulse_us", 500))
        self.max_pulse_us = int(rospy.get_param("~max_pulse_us", 2500))
        self.open_angle_deg = float(rospy.get_param("~open_angle_deg", 20.0))
        self.close_angle_deg = float(rospy.get_param("~close_angle_deg", 95.0))
        self.settle_time_s = float(rospy.get_param("~settle_time_s", 0.4))
        self.invert_direction = bool(rospy.get_param("~invert_direction", False))
        self.open_on_shutdown = bool(rospy.get_param("~open_on_shutdown", True))

        self.backend = self._build_backend()

        self.current_angle = self.open_angle_deg
        self.holding_object = False

        self.pub_angle = rospy.Publisher("/gripper/state", Float32, queue_size=10, latch=True)
        self.pub_hold = rospy.Publisher("/gripper/holding_object", Bool, queue_size=10, latch=True)

        self.service = rospy.Service("/gripper/set", SetGripper, self._set_cb)

        ok, msg = self._send_angle(self.current_angle)
        if not ok:
            rospy.logwarn("initial gripper open command failed: %s", msg)
        self._publish_state()

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("gripper_controller ready (backend=%s)", self.backend.mode)

    def _build_backend(self):
        if self.backend_name in ("mock", "test"):
            return MockBackend()
        if self.backend_name in ("pigpio", "local_pwm"):
            return PigpioBackend(self.pwm_channel, self.pwm_frequency_hz)
        if self.backend_name in ("serial", "arduino_serial", "nano_serial"):
            return SerialBackend(
                self.serial_port,
                self.serial_baud_rate,
                self.serial_timeout_s,
                self.startup_delay_s,
            )
        raise RuntimeError(f"unsupported gripper backend '{self.backend_name}'")

    def _angle_to_pulse(self, angle_deg):
        angle_deg = max(0.0, min(180.0, angle_deg))
        ratio = angle_deg / 180.0
        pulse = self.min_pulse_us + ratio * (self.max_pulse_us - self.min_pulse_us)
        return int(round(pulse))

    def _normalized_angle(self, angle_deg):
        angle = max(0.0, min(180.0, float(angle_deg)))
        if self.invert_direction:
            angle = 180.0 - angle
        return angle

    def _send_line(self, line):
        return self.backend.command(line, self.command_timeout_s)

    def _send_angle(self, angle_deg):
        angle = self._normalized_angle(angle_deg)
        pulse = self._angle_to_pulse(angle)

        if self.backend.mode == "pigpio":
            ok, msg = self._send_line(f"PULSE_US {pulse}")
        else:
            ok, msg = self._send_line(f"ANGLE {angle:.1f}")

        if ok:
            self.current_angle = angle
        return ok, msg

    def _publish_state(self):
        self.pub_angle.publish(Float32(data=float(self.current_angle)))
        self.pub_hold.publish(Bool(data=bool(self.holding_object)))

    def _set_cb(self, req):
        cmd = (req.command or "").strip().lower()
        with self.lock:
            if cmd == "open":
                ok, msg = self._send_angle(self.open_angle_deg)
                if ok:
                    self.current_angle = self._normalized_angle(self.open_angle_deg)
                    self.holding_object = False
            elif cmd == "close":
                ok, msg = self._send_angle(self.close_angle_deg)
                if ok:
                    self.current_angle = self._normalized_angle(self.close_angle_deg)
                    self.holding_object = True
            elif cmd == "angle":
                ok, msg = self._send_angle(float(req.angle_deg))
            else:
                return SetGripperResponse(success=False, message="command must be open|close|angle")

            if not ok:
                return SetGripperResponse(success=False, message=msg)

            rospy.sleep(self.settle_time_s)
            self._publish_state()
            return SetGripperResponse(success=True, message=msg)

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
