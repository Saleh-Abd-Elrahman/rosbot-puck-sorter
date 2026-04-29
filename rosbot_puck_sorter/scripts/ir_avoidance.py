#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import math

class IRAvoidanceSingle:

    def __init__(self):
        rospy.init_node('ir_avoidance_node')
        rospy.loginfo("IR Avoidance Node Started")

        # Safe distance to obstacle (meters)
        self.safe_distance = 0.5

        # Sensor reading
        self.front_range = float('inf')

        # Subscriber to front IR sensor
        rospy.Subscriber('/range/fl', Range, self.front_callback)

        # Publisher to cmd_vel
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Loop rate
        self.rate = rospy.Rate(10)

    def front_callback(self, msg):
        # Update front sensor value
        self.front_range = msg.range

    def rotate_90_degrees(self, clockwise=False):
        """Rotate the robot ~90 degrees in place."""
        twist = Twist()
        angular_speed = 1.0  # rad/s
        angle = math.pi / 2  # 90 degrees

        twist.angular.z = -angular_speed if clockwise else angular_speed
        twist.linear.x = 0.0

        duration = angle / angular_speed
        rospy.loginfo(f"Rotating {'clockwise' if clockwise else 'counterclockwise'} 90° for {duration:.2f}s")

        t0 = rospy.Time.now().to_sec()
        current_time = 0

        while current_time < duration and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_time = t1 - t0

        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Rotation complete")

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()
            forward_speed = 0.2
            twist.angular.z = 0.0

            rospy.loginfo(f"Front sensor: {self.front_range:.2f} m")

            # Emergency: obstacle too close → rotate 90°
            if self.front_range < 0.2:
                rospy.logwarn("Obstacle VERY close! Performing 90° rotation")
                self.rotate_90_degrees(clockwise=False)
                continue  # skip normal forward motion

            # Normal avoidance: slow down if approaching obstacle
            if self.front_range < self.safe_distance:
                # Proportional speed reduction
                forward_speed = 0.2 * (self.front_range / self.safe_distance)
                forward_speed = max(forward_speed, 0.05)  # minimum speed

            twist.linear.x = forward_speed

            rospy.loginfo(f"Publishing → linear: {twist.linear.x:.2f}, angular: {twist.angular.z:.2f}")
            self.cmd_pub.publish(twist)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = IRAvoidanceSingle()
        node.run()
    except rospy.ROSInterruptException:
        pass

