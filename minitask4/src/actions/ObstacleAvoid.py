from geometry_msgs.msg import Twist, Vector3, Pose
from IAction import IAction
from common import MoveParam
from Global import *

class ObstacleAvoid(IAction):
    def __init__(self, topicMsg):
        super(ObstacleAvoid, self).__init__("ObstacleAvoid")
        self.topicMsg = topicMsg
        self.pub = self.topicMsg.pub

    def execute(self):
        """Execute the action"""
        if self.topicMsg.lastState != OBSTACLE_AVOID: 
            self.pub.publish(Twist())

        vel = Twist(angular=Vector3(0, 0, MoveParam.angular_speed))
        self.pub.publish(vel)
        self.topicMsg.rate.sleep()
        return True
