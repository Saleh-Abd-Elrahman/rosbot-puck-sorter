#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

from rosbot_puck_sorter.srv import SetGripper

from common import load_script_module, safe_shutdown, wait_for


def main():
    rospy.init_node("test_gripper_controller")

    rospy.set_param("~backend", "mock")
    rospy.set_param("~pwm_channel", 0)
    rospy.set_param("~pwm_frequency_hz", 50)
    rospy.set_param("~min_pulse_us", 500)
    rospy.set_param("~max_pulse_us", 2500)
    rospy.set_param("~open_angle_deg", 15.0)
    rospy.set_param("~close_angle_deg", 90.0)
    rospy.set_param("~settle_time_s", 0.05)
    rospy.set_param("~assume_holding_without_feedback", True)

    module = load_script_module("gripper_controller.py", "gripper_controller_test_mod")
    module.GripperController()

    hold = {"value": None}

    def hold_cb(msg):
        hold["value"] = bool(msg.data)

    rospy.Subscriber("/gripper/holding_object", Bool, hold_cb, queue_size=10)

    rospy.wait_for_service("/gripper/set", timeout=5.0)
    gripper_srv = rospy.ServiceProxy("/gripper/set", SetGripper)

    rsp = gripper_srv(command="open", angle_deg=0.0)
    if not rsp.success:
        raise RuntimeError(f"open failed: {rsp.message}")

    wait_for(lambda: hold["value"] is False, timeout_s=2.0, desc="holding=false after open")

    rsp = gripper_srv(command="close", angle_deg=0.0)
    if not rsp.success:
        raise RuntimeError(f"close failed: {rsp.message}")

    wait_for(lambda: hold["value"] is True, timeout_s=2.0, desc="holding=true after close")

    print("PASS: gripper controller service test")
    safe_shutdown()


if __name__ == "__main__":
    main()
