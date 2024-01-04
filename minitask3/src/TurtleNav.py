#!/usr/bin/env python

from colorsys import ONE_SIXTH
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose
import math
import random
import traceback
from actions.RandomWalk import RandomWalk
from actions.ObstacleAvoid import ObstacleAvoid
from actions.RhWallFollow import RhWallFollow
from actions.RunToGreen import RunToGreen
from Global import *
from common import MoveParam
from common.DataStruct import TopicMsg
from subscriber.OdomSubscribe import OdomSubscribe
from subscriber.ScanSubscribe import ScanSubscribe
from subscriber.ImageRawSubscribe import ImageRawSubscribe
from Logger import logger

class TurtleNav:
    def __init__(self):
        rospy.init_node('TurtleNav', anonymous=True)
        self.topicMsg = TopicMsg()
        self.topicMsg.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub = self.topicMsg.pub
        rospy.on_shutdown(self.onShutdown)
        self.rate = self.topicMsg.rate = rospy.Rate(300)
        self.state = None

        #listener
        OdomSubscribe(rospy, self.topicMsg)
        ScanSubscribe(rospy, self.topicMsg)
        ImageRawSubscribe(rospy, self.topicMsg)

        # action instance
        self.randomWalk = RandomWalk(self.topicMsg)
        self.obstacleAvoid = ObstacleAvoid(self.topicMsg)
        self.rhWallFollow = RhWallFollow(self.topicMsg)
        self.runToGreen = RunToGreen(self.topicMsg)
        self.moveParam = MoveParam()

    def isObstacle(self):
        for x in self.topicMsg.forwardList:
            logger.debug(x)
            if x < MoveParam.obstacle_dis and not x == 0:
                return True
        return False

    def isRhWallFollow(self):
        if (self.topicMsg.rightLaser < MoveParam.wallEntryThres) and\
           ((self.topicMsg.rightBackLaser < MoveParam.wallEntryRBThres)\
           or (self.topicMsg.rightFrontLaser < MoveParam.wallEntryRFThres)):
                self.state = RH_WALL_FOLLOW
                logger.debug("in wall status!")
                return True
        else:
            return False

    def isRunToGreen(self):
        if (MoveParam.runToGreen_switch == 1) and\
                self.topicMsg.toGreenDirection != GREEN_NOTHING:
            return True
        else:
            return False
    
    def onShutdown(self):
        self.pub.publish(Twist())

    def stateCheck(self):
        self.topicMsg.lastState = self.state
        if self.isObstacle():
            self.state = OBSTACLE_AVOID
        elif self.isRunToGreen():
            self.state = RUN_TO_GREEN
        elif self.isRhWallFollow():
            self.state = RH_WALL_FOLLOW
        else:
            self.state = RANDOM_WALK
    
    def logicAfterChecking(self):
        
        if self.topicMsg.lastState == OBSTACLE_AVOID and\
                self.state != OBSTACLE_AVOID:
            self.pub.publish(Twist(angular=Vector3(0, 0, 0)))

        if self.topicMsg.lastState != RANDOM_WALK and\
                self.state == RANDOM_WALK:
            self.topicMsg.distanceTravelled = 0

        if self.topicMsg.lastState != self.state:
            logger.info("state: " + self.state)
         
    def turtleNav(self):
        while rospy.Time.now().to_sec() == 0:
            pass

        while not rospy.is_shutdown():          
            # every loop has 3 steps, every loop only has several milliseconds.
            # step 1
            self.stateCheck()
            # step 2
            self.logicAfterChecking()

            # step 3 
            if self.state == OBSTACLE_AVOID:
                self.obstacleAvoid.execute()
            elif self.state == RUN_TO_GREEN:
                self.runToGreen.execute()
            elif self.state == RH_WALL_FOLLOW:
                self.rhWallFollow.execute()
            elif self.state == RANDOM_WALK:
                if MoveParam.found_green == 1:
                    MoveParam.found_green = 0
                self.randomWalk.execute()
            else:
                self.randomWalk.execute()
        rospy.spin()


if __name__ == '__main__':
    try:
        turtleBot = TurtleNav()
        turtleBot.turtleNav()
    except Exception as e:
        rospy.loginfo("Exception entered!")
        rospy.loginfo(e.args)
        rospy.loginfo(traceback.format_exc())
