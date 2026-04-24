#!/usr/bin/env python3
import importlib.util
import math
import os
import sys
import time

import rospy


def package_root_from_here():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_script_module(script_filename, module_name):
    script_path = os.path.join(package_root_from_here(), "scripts", script_filename)
    scripts_dir = os.path.dirname(script_path)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wait_for(predicate, timeout_s=5.0, period_s=0.05, desc="condition"):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        if predicate():
            return True
        time.sleep(period_s)
    raise RuntimeError(f"timeout waiting for {desc}")


def yaw_to_quat(yaw):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def safe_shutdown(reason="test complete"):
    try:
        rospy.signal_shutdown(reason)
    except Exception:
        pass
