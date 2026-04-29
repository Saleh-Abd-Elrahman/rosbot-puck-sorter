#!/usr/bin/env python3
"""
Open the gripper (publish 0 to /servo) and stop the robot (zero Twist on /cmd_vel).
Usage:
  rosrun my_python_pkg gripper_reset.py            # open + stop
  rosrun my_python_pkg gripper_reset.py close      # close (170)
  rosrun my_python_pkg gripper_reset.py 90         # any angle 0..180
"""
import sys
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist


def main():
    angle = 30
    if len(sys.argv) > 1:
        a = sys.argv[1].lower()
        if a in ("open",):
            angle = 0
        elif a in ("close", "closed"):
            angle = 170
        else:
            angle = int(a)
    angle = max(0, min(180, angle))

    rospy.init_node("gripper_reset", anonymous=True)
    servo_pub = rospy.Publisher("/servo", UInt16, queue_size=1, latch=True)
    cmd_pub   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.5)

    servo_pub.publish(UInt16(data=angle))
    cmd_pub.publish(Twist())
    rospy.loginfo("gripper -> %d, cmd_vel -> 0", angle)
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
