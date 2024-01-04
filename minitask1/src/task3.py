#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import pi
import traceback
import matplotlib.pyplot as plt
import tf

class Vector3D():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y 
        self.z = z

class DrawSquare():
    def __init__(self):
        self.linear_speed = 0.05
        self.total_distance = 1.0 #m

        self.angular_speed = 0.05
        self.total_angle = pi/2

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('draw_square', anonymous=True)
        self.rate_num = 300
        self.rate = rospy.Rate(self.rate_num)
        self.pose = Vector3D(0, 0, 0)

        self.plot_x = []
        self.plot_y = []

    def drawLine(self, speed, length):
        twist = Twist()
        twist.linear.x = speed
        linear_duration = length/speed
        ticks = int(linear_duration * self.rate_num)

        for i in range(ticks): 
            self.pub.publish(twist)
            self.rate.sleep()

        self.pub.publish(Twist())

    def drawRotate(self, angular_speed, total_angle):
        twist = Twist()
        twist.angular.z = angular_speed
        rotate_duration = total_angle / angular_speed
        ticks = int(rotate_duration * self.rate_num)

        for i in range(ticks): 
            self.pub.publish(twist)
            self.rate.sleep()

        self.pub.publish(Twist())

    def odomCallback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.plot_x.append(self.pose.x)
        self.plot_y.append(self.pose.y)
        print("x: " + str(self.pose.x) + "y: " + str(self.pose.y))

    def odomListener(self):
        rospy.Subscriber("/odom", Odometry, self.odomCallback)

    def draw(self):
        self.drawLine(self.linear_speed, self.total_distance)
        self.drawRotate(self.angular_speed, self.total_angle)
        self.drawLine(self.linear_speed, self.total_distance)
        self.drawRotate(self.angular_speed, self.total_angle)
        self.drawLine(self.linear_speed, self.total_distance)
        self.drawRotate(self.angular_speed, self.total_angle)
        self.drawLine(self.linear_speed, self.total_distance)

    def shutdown(self):
        self.pub.publish(Twist())

    def plot_trajectory(self):
        plt.plot(self.plot_x, self.plot_y)
        plt.show()

if __name__ == '__main__':
    try:
        run = DrawSquare()
        run.odomListener()
        run.draw()
        run.plot_trajectory()
    except Exception as e:
        rospy.loginfo("drawSquare failed!");
        rospy.loginfo(e.args);
        rospy.loginfo(traceback.format_exc());
