#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("drive_forward_backward")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10)

cmd = Twist()
def stop():
    cmd.linear.x = 0.0
    cmd.linear.z = 0.0
    pub.publish(cmd)

cmd.linear.x = 0.2
cmd.angular.z = 0.0
start = rospy.Time.now()

while rospy.Time.now() - start < rospy.Duration(1.0):
    pub.publish(cmd)
    rate.sleep()

cmd.linear.x = 0.0
cmd.angular.z = 0.5
start = rospy.Time.now()

stop()
rospy.sleep(1)

cmd.linear.x = 0.2
start = rospy.Time.now()

while rospy.Time.now() - start < rospy.Duration(1.0):
    pub.publish(cmd)
    rate.sleep()
stop()
