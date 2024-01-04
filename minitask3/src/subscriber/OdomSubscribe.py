import rospy
import math
import tf
from nav_msgs.msg import Odometry
from common.DataStruct import TopicMsg
from geometry_msgs.msg import Twist, Vector3, Pose
from Logger import logger

class OdomSubscribe():
    def __init__(self, rospy, topicMsg):
        self.rospy = rospy
        self.topicMsg = topicMsg
        self.rospy.Subscriber("odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # why was it pose.theta and so on (error: pose object has no attribute theta)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.topicMsg.currentX == None:
            self.topicMsg.currentX = self.x
        if self.topicMsg.currentY == None:
            self.topicMsg.currentY = self.y
        if self.topicMsg.currentTheta == None:
            self.topicMsg.currentTheta = self.theta

        changex = self.x - self.topicMsg.currentX
        changey = self.y - self.topicMsg.currentY
        changeDis = math.sqrt((changex * changex) + (changey * changey))
        self.topicMsg.distanceTravelled += changeDis

        self.topicMsg.currentX = self.x
        self.topicMsg.currentY = self.y
        self.topicMsg.currentTheta = self.theta
