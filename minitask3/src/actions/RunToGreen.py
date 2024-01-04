from IAction import IAction
from geometry_msgs.msg import Twist, Vector3
from common import MoveParam
from Global import *

class RunToGreen(IAction):
    def __init__(self, topicMsg):
        super(RunToGreen, self).__init__("RunToGreen")
        self.topicMsg = topicMsg
        self.pub = self.topicMsg.pub

    def execute(self):
        """Execute the action"""
        vel = None

        vel = self.topicMsg.greenVel
         
        self.pub.publish(vel)
        self.topicMsg.rate.sleep()
        return
