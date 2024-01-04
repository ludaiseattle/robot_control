import rospy
import math
from math import cos, sin
from common.OccupiedGrid import OccupiedGrid
from common.DataStruct import TopicMsg
from Logger import logger
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import thread

class ScanSubscribe():
    def __init__(self, rospy, topicMsg, occupancyGrid):
        self.rospy = rospy
        self.topicMsg = topicMsg
        self.occuGrid = occupancyGrid
        
        self.rospy.Subscriber("scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        self.topicMsg.frontLaser = msg.ranges[0]
        self.topicMsg.rightLaser = msg.ranges[270]
        self.topicMsg.leftLaser = msg.ranges[90]
        self.topicMsg.rightBackLaser = msg.ranges[250]
        self.topicMsg.rightFrontLaser = msg.ranges[290]
        self.topicMsg.laser_range_min = msg.range_min
        self.topicMsg.laser_range_max = msg.range_max
        logger.debug("laser range: " + str(self.topicMsg.laser_range_min) +\
                "-" + str(self.topicMsg.laser_range_max))

        if len(self.topicMsg.forwardList) > 0:
            del self.topicMsg.forwardList[:]

        for i in range((0 + self.topicMsg.forwardFOV/2), (-1 - self.topicMsg.forwardFOV/2), -1):
            self.topicMsg.forwardList.append(msg.ranges[i%360])

        self.topicMsg.scan_msg = msg
        #self.printRadar(msg)

    def printRadar(self, msg):
        for i in range(0, 360):
            dis = msg.ranges[i]
            ang = i
            print ("[ang: " + str(ang) + " dis: " + str(dis) + "]",)
        print("\n")

