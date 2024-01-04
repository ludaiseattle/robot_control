import math
import tf
import message_filters
from nav_msgs.msg import Odometry
from common.DataStruct import TopicMsg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Vector3, Pose
from common import OccupiedGridParams as params
from common.OccupiedGrid import OccupiedGrid
from Logger import logger
from sensor_msgs.msg import LaserScan

class SubSyn():
    def __init__(self, topicMsg):
        self.topicMsg = topicMsg
        self.odom = message_filters.Subscriber("odom", Odometry)
        self.scan = message_filters.Subscriber("scan", LaserScan)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.odom, self.scan], queue_size = 10, slop = 0.05)
        #self.tss = message_filters.TimeSynchronizer([self.odom, self.scan], queue_size = 10)
        self.tss.registerCallback(self.callback)

    def callback(self, odom_msg, scan_msg):
        self.odomCallback(odom_msg)
        self.scanCallback(scan_msg)

    def odomCallback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # why was it pose.theta and so on (error: pose object has no attribute theta)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        logger.debug("robot [pose_x:" + str(self.x) + " pose_y:" +\
                str(self.y) + " yaw(theta):" + str(self.theta) + "]")

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

        self.topicMsg.odom_msg = msg

    def scanCallback(self, msg):
        self.topicMsg.frontLaser = msg.ranges[0]
        self.topicMsg.rightLaser = msg.ranges[270]
        self.topicMsg.leftLaser = msg.ranges[90]
        self.topicMsg.rightBackLaser = msg.ranges[240]
        self.topicMsg.rightFrontLaser = msg.ranges[300]
        self.topicMsg.laser_range_min = msg.range_min
        self.topicMsg.laser_range_max = msg.range_max
        logger.debug("laser range: " + str(self.topicMsg.laser_range_min) +\
                "-" + str(self.topicMsg.laser_range_max))

        if len(self.topicMsg.forwardList) > 0:
            del self.topicMsg.forwardList[:]

        for i in range((0 + self.topicMsg.forwardFOV/2), (-1 - self.topicMsg.forwardFOV/2), -1):
            self.topicMsg.forwardList.append(msg.ranges[i%360])

        self.topicMsg.scan_msg = msg
