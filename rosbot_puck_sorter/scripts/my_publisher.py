#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("cmd_vel_pub")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10)

cmd = Twist()
cmd.linear.x = 0.2
cmd.angular.z = 0.5

while not rospy.is_shutdown():
    pub.publish(cmd)
    rate.sleep()
