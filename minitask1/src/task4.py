#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import traceback
import tf
from geometry_msgs.msg import Pose
import time
import matplotlib.pyplot as plt

class Vector3D():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y 
        self.z = z
    

class RunInSquare():
    def __init__(self):
        self.linear_speed = 0.05
        self.angular_speed = 0.05

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('run_in_square', anonymous=True)
        self.rate_num = 300
        self.rate = rospy.Rate(self.rate_num)
        self.current_pose = Vector3D(0, 0, 0)
        self.plot_x = []
        self.plot_y = []

    def isArrived(self, origin, current, action):
        diff = action[0]
        axis = action[1]
        if axis == 'x':
            if abs(current.x - origin.x) >= diff:
                return True
            else:
                return False
        elif axis == 'y':
            if abs(current.y - origin.y) >= diff: 
                return True
            else:
                return False
        elif axis == 'z':
            if abs(current.z - origin.z) >= diff: 
                return True
            else:
                return False

    def MoveTo(self, action):
        twist = Twist()
        origin_pose = Vector3D(self.current_pose.x, self.current_pose.y, self.current_pose.z)

        while True: 
            if self.isArrived(origin_pose, self.current_pose, action):
                break
            self.plot_x.append(self.current_pose.x)
            self.plot_y.append(self.current_pose.y)
            if action[1] == 'x' or action[1] == 'y':
                twist.linear.x = self.linear_speed
            else:
                twist.angular.z = self.angular_speed
            self.pub.publish(twist)
            self.rate.sleep()
            
        self.pub.publish(Twist())

    def odomCallback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.current_pose.z = yaw
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        #print("odomCallback: current_x: " + str(self.current_pose.x) + " current_y: " + 
                   # str(self.current_pose.y) + " current_z: " + str(self.current_pose.z))
    
    def odomListener(self):
        rospy.Subscriber("/odom", Odometry, self.odomCallback)

    def run(self):
        action_list = [(1, 'x'), (pi/2, 'z'), (1, 'y'), (pi/2, 'z'), \
                (1, 'x'), (pi/2, 'z'), (1, 'y')]

        for action in action_list:
            self.MoveTo(action)

    def shutDown(self):
        self.pub.publish(Twist())

    def plot_trajectory(self):
        plt.plot(self.plot_x, self.plot_y)
        plt.show()

if __name__ == '__main__':
    try:
        run = RunInSquare()
        run.odomListener()
        time.sleep(1)
        run.run()
        run.shutDown()
        run.plot_trajectory()

    except Exception as e:
        rospy.loginfo("drawSquare failed!");
        run.pub.publish(Twist())
        rospy.loginfo(e.args);
        rospy.loginfo(traceback.format_exc());
