from IAction import IAction
from common import MoveParam
from Global import *
import math
from geometry_msgs.msg import Twist, Vector3, Pose
import random
from Logger import logger

GO_STRAIGHT = 1
ROTATE = 2

class RandomWalk(IAction):

    def __init__(self, topicMsg):
        super(RandomWalk, self).__init__("RandomWalk")
        self.topicMsg = topicMsg
        self.pub = self.topicMsg.pub
        self.workMode = GO_STRAIGHT 

    def goStraight(self):
        if self.topicMsg.distanceTravelled < MoveParam.RAND_DIS:
            logger.debug("travelled: " + str(self.topicMsg.distanceTravelled) +\
                    " dis: " + str(MoveParam.RAND_DIS))
            vel = Twist(linear=Vector3(MoveParam.linear_speed, 0, 0), angular=(Vector3(0, 0, - 3 * MoveParam.angular_speed)))
            self.pub.publish(vel)
            self.topicMsg.rate.sleep()
        else:
            self.topicMsg.distanceTravelled = 0
            self.workMode = ROTATE
            # MoveParam.desiredTheta = math.radians(random.randint(0, 360))
            MoveParam.desiredTheta = math.radians(math.pi/2)
            if MoveParam.desiredTheta > math.pi:
                MoveParam.desiredTheta = math.pi - MoveParam.desiredTheta

    def rotate(self):
        logger.debug("in rotate mode!")
        if self.topicMsg.currentTheta < MoveParam.desiredTheta:
            if MoveParam.desiredTheta == math.pi and self.topicMsg.currentTheta < 0:
                MoveParam.desiredTheta = -math.pi
                self.workMode = GO_STRAIGHT
                return
            vel = Twist(angular=Vector3(0.03, 0, MoveParam.angular_speed))
            self.pub.publish(vel)
            self.topicMsg.rate.sleep()
        else:
            self.workMode = GO_STRAIGHT

    def execute(self):
        """Execute the action"""
        if self.workMode == GO_STRAIGHT:
            self.goStraight()
        elif self.workMode == ROTATE:
            self.rotate()
        else:
            self.workMode = GO_STRAIGHT



