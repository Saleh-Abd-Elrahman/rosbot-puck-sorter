#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class CornerParking:

    def __init__(self):
        rospy.init_node('corner_parking_node')
        rospy.loginfo("Corner Parking Node Started")

        self.front_target = 0.25  # desired front distance in meters
        self.back_target = 0.25   # desired back distance in meters

        self.fr_range = float('inf')  # front right sensor
        self.fl_range = float('inf')  # back left sensor

        rospy.Subscriber('/range/fr', Range, self.fr_callback)
        rospy.Subscriber('/range/fl', Range, self.fl_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def fr_callback(self, msg):
        self.fr_range = msg.range

    def fl_callback(self, msg):
        self.fl_range = msg.range

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()

            rospy.loginfo(f"FR (front): {self.fr_range:.3f} m | FL (back): {self.fl_range:.3f} m")

            front_error = self.fr_range - self.front_target
            back_error = self.fl_range - self.back_target

            tolerance = 0.05  # 5 cm tolerance

            if abs(front_error) < tolerance and abs(back_error) < tolerance:
                rospy.loginfo("Parked successfully! Stopping.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                break

            # Linear speed to approach front wall
            kp_linear = 0.6
            linear_speed = kp_linear * front_error
            linear_speed = max(min(linear_speed, 0.15), -0.15)

            # Angular speed to rotate robot into corner on the RIGHT side
            # Since the robot should park on the right corner,
            # the robot needs to turn **right** when back sensor distance > front sensor distance
            angle_error = back_error - front_error
            kp_angular = 2.0  # stronger turning for sharper cornering

            # Positive angular.z → rotate counterclockwise (left turn)
            # Negative angular.z → rotate clockwise (right turn)
            angular_speed = -kp_angular * angle_error  # invert sign for right turn correction
            angular_speed = max(min(angular_speed, 0.8), -0.8)

            # If too close to front wall, slow down linear speed aggressively
            if front_error < 0:
                linear_speed = max(linear_speed, -0.05)  # allow small reverse to adjust if too close

            twist.linear.x = linear_speed
            twist.angular.z = angular_speed

            rospy.loginfo(f"Commanding linear.x: {linear_speed:.3f} m/s, angular.z: {angular_speed:.3f} rad/s")

            self.cmd_pub.publish(twist)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = CornerParking()
        node.run()
    except rospy.ROSInterruptException:
        pass

