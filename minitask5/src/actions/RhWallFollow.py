from geometry_msgs.msg import Twist, Vector3, Pose
from IAction import IAction
from common import MoveParam
from Global import *

class RhWallFollow(IAction):
    def __init__(self, topicMsg):
        super(RhWallFollow, self).__init__("RhWallFollow")
        self.topicMsg = topicMsg
        self.pub = self.topicMsg.pub

    def execute(self):
        """Execute the action"""
        if self.topicMsg.rightFrontLaser > MoveParam.wallEntryRFThres:
            self.pub.publish(Twist())
            self.pub.publish(Twist(angular=Vector3(0, 0, 13.5 * -MoveParam.angular_speed)))
            print("turn right")
            self.topicMsg.rate.sleep()
        else:
            if self.topicMsg.rightLaser > MoveParam.wallMaxThres:
                self.pub.publish(Twist())
                self.pub.publish(Twist(angular=Vector3(0, 0, -MoveParam.angular_speed)))
                self.topicMsg.rate.sleep()
            elif self.topicMsg.rightLaser < MoveParam.wallMinThres:
                self.pub.publish(Twist())
                self.pub.publish(Twist(angular=Vector3(0, 0, MoveParam.angular_speed)))
                self.topicMsg.rate.sleep()

            vel = Twist(linear=Vector3(MoveParam.linear_speed, 0, 0))
            self.pub.publish(vel)
            self.topicMsg.rate.sleep()
        return True
