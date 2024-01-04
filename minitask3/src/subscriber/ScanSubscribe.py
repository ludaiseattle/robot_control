import rospy
from common.DataStruct import TopicMsg
from sensor_msgs.msg import LaserScan
class ScanSubscribe():
    def __init__(self, rospy, topicMsg):
        self.rospy = rospy
        self.topicMsg = topicMsg
        self.rospy.Subscriber("scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        self.topicMsg.frontLaser = msg.ranges[0]
        self.topicMsg.rightLaser = msg.ranges[270]
        self.topicMsg.leftLaser = msg.ranges[90]
        self.topicMsg.rightBackLaser = msg.ranges[240]
        self.topicMsg.rightFrontLaser = msg.ranges[300]

        if len(self.topicMsg.forwardList) > 0:
            del self.topicMsg.forwardList[:]

        for i in range((0 + self.topicMsg.forwardFOV/2), (-1 - self.topicMsg.forwardFOV/2), -1):
            self.topicMsg.forwardList.append(msg.ranges[i%360])
