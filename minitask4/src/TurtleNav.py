#!/usr/bin/env python

from colorsys import ONE_SIXTH
import rospy
import math
import tf
import random
import traceback
import message_filters
from Global import *
from Logger import logger
from common import MoveParam
from actions.RandomWalk import RandomWalk
from actions.ObstacleAvoid import ObstacleAvoid
from actions.RhWallFollow import RhWallFollow
from actions.MoveToGoal import MoveToGoal
from common.DataStruct import TopicMsg
from common.OccupiedGrid import OccupiedGrid
from subscriber.OdomSubscribe import OdomSubscribe
from subscriber.ScanSubscribe import ScanSubscribe
from subscriber.SubSyn import SubSyn
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from listener.DrawCostMap import DrawCostMap
from publisher.CostMapPub import CostMapPub
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class TurtleNav:
    def __init__(self):
        rospy.init_node('TurtleNav', anonymous=True)
        self.topicMsg = TopicMsg()
        self.topicMsg.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub = self.topicMsg.pub
        rospy.on_shutdown(self.onShutdown)
        self.rate = self.topicMsg.rate = rospy.Rate(300)
        self.state = None
        self.des = [(-1.0, 4.0), (4.0,1.0), (1.0, 3.0), (6.0, -3.0), (-6.5, 3.0), (6.5, 2.0)]
        #self.des = [(-1.0, 4.0)]
        self.occuGrid = OccupiedGrid()
        self.occuGrid.setTopicMsg(self.topicMsg)

        # subscriber
        #OdomSubscribe(rospy, self.topicMsg, self.occuGrid)
        #ScanSubscribe(rospy, self.topicMsg, self.occuGrid)
        SubSyn(self.topicMsg)

        # action instance
        self.randomWalk = RandomWalk(self.topicMsg)
        self.obstacleAvoid = ObstacleAvoid(self.topicMsg)
        self.rhWallFollow = RhWallFollow(self.topicMsg)
        self.moveToGoal = MoveToGoal(self.topicMsg)
        self.moveParam = MoveParam()
        
        # listener
        self.t_drawCostMap = DrawCostMap(self.topicMsg, self.occuGrid)

        # publisher
        self.t_costMapPub = CostMapPub(rospy, self.occuGrid) 


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
    
    def onShutdown(self):
        self.pub.publish(Twist())
        self.beforeStop()

    def stateCheck(self):
        self.topicMsg.lastState = self.state
        if self.isObstacle():
            self.state = OBSTACLE_AVOID
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

        self.t_drawCostMap.start()
        self.t_costMapPub.start()

        # for i in self.des:
            
        #     logger.info("Now move to: (" + str(i[0]) + ", " + str(i[1]) + ")")
        #     pose = Pose()
        #     pose.position = Point(i[0], i[1], 0)
        #     pose.orientation.x = 0.0
        #     pose.orientation.y = 0.0
        #     pose.orientation.z = 0.0
        #     pose.orientation.w = 1.0
        #     ret = self.moveToGoal.execute(pose)

        #     if ret:
        #         logger.info("destination (" + str(i[0]) +\
        #                 ", " + str(i[1]) + ") arrived!")
        #     else:
        #         logger.error("destination (" + str(i[0]) +\
        #                 ", " + str(i[1]) + ") couldn't arrived, terminate!")
        #         break

        while not rospy.is_shutdown():           

            if not -1 in self.occuGrid.getData():
                rospy.signal_shutdown("map complete")
                break
            # every loop has 3 steps, every loop only has several milliseconds.
            # step 1
            self.stateCheck()
            # step 2
            self.logicAfterChecking()

            # step 3 
            if self.state == OBSTACLE_AVOID:
                self.obstacleAvoid.execute()
            elif self.state == RH_WALL_FOLLOW:
                self.rhWallFollow.execute()
            elif self.state == RANDOM_WALK:
                self.randomWalk.execute()
            else:
                self.randomWalk.execute()
        rospy.spin()

        

    def beforeStop(self):
        self.t_costMapPub.stop()
        self.t_costMapPub.join()
        self.t_drawCostMap.stop()
        self.t_drawCostMap.join()
        #self.occuGrid.drawCostMap()
        

if __name__ == '__main__':
    try:
        turtleBot = TurtleNav()
        turtleBot.turtleNav()
    except Exception as e:
        rospy.loginfo("Exception entered!")
        rospy.loginfo(e.args)
        rospy.loginfo(traceback.format_exc())
