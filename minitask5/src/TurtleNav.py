#!/usr/bin/env python

import rospy
import math
import tf
import random
import traceback
import time
import message_filters
from PIL import Image
from pylab import *
from Global import *
from Logger import logger
from common import MoveParam
from colorsys import ONE_SIXTH
from actions.RandomWalk import RandomWalk
from actions.ObstacleAvoid import ObstacleAvoid
from actions.RhWallFollow import RhWallFollow
from actions.MoveToGoal import MoveToGoal
from common.DataStruct import TopicMsg
from common.DataStruct import PathMsg
from common.OccupiedGrid import OccupiedGrid
from subscriber.OdomSubscribe import OdomSubscribe
from subscriber.ScanSubscribe import ScanSubscribe
from subscriber.SubSyn import SubSyn
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from subscriber.ImageRawSubscribe import ImageRawSubscribe
from nav_msgs.msg import OccupancyGrid
from listener.DrawCostMap import DrawCostMap
from publisher.CostMapPub import CostMapPub
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from path_finding.GlobalPath import GlobalPath
from path_finding.LocalPath import LocalPath
from path_finding.MoveToChecking import MoveToChecking

class TurtleNav:
    def __init__(self):
        rospy.init_node('TurtleNav', anonymous=True)
        self.topicMsg = TopicMsg()
        self.pathMsg = PathMsg()
        self.topicMsg.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub = self.topicMsg.pub
        rospy.on_shutdown(self.onShutdown)
        self.rate = self.topicMsg.rate = rospy.Rate(300)
        self.state = None
        self.checkpoints = [(-1.168, -1.046), (0.366, -0.866), (0.438, -3.297)\
                ,(2.593, -3.192), (2.691, 1.025), (5.934, 0.931), (5.662, -1.965)\
                ,(5.735, 0.984), (0.100, 0.936), (0.194, 4.188), (1.538, 4.207)\
                ,(1.403, 2.263), (4.077, 2.491)]
        self.pathList = []
        self.green_found = 0
        self.red_found = 0
        self.occuGrid = OccupiedGrid(self.topicMsg)
        self.occuGrid.setTopicMsg(self.topicMsg)

        # subscriber
        #OdomSubscribe(rospy, self.topicMsg, self.occuGrid)
        #ScanSubscribe(rospy, self.topicMsg, self.occuGrid)
        #commenting out whilst testing image subscribe
        SubSyn(self.topicMsg)
        ImageRawSubscribe(rospy, self.topicMsg, self.occuGrid)

        #path finding
        self.gPathFinding = GlobalPath(self.pathMsg, self.occuGrid)
        self.lPathFinding = LocalPath(self.topicMsg, self.pathMsg, self.occuGrid)
        #self.moveToChecking = MoveToChecking(self.topicMsg)

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

    def getPointAvailable(self, x, y):
        index = self.occuGrid.to_index_api(x, y)
        value = self.occuGrid.grid.data[index]
        if value == 0:
            return x, y

        length = len(self.occuGrid.grid.data)
        for ex in range(x-7, x+7):
            for ey in range(y-7, y+7):
                index = self.occuGrid.to_index_api(ex, ey)
                if index < length:
                    value = self.occuGrid.grid.data[index]
                    if value == 0:
                        return ex, ey 
        return None, None

    def moveToTarget(self):
        print(self.topicMsg.green_image_goals)
        print(type(self.topicMsg.green_image_goals))
        
        while len(self.topicMsg.green_image_goals) > 0:
            #logger.info("Now move to green target:" + str(self.topicMsg.green_image_goals[i][0])\
            #        + "," + str(self.topicMsg.green_image_goals[i][1]))
            aPoint = self.getPointAvailable(self.topicMsg.green_image_goals[0][0], self.topicMsg.green_image_goals[0][1])
            if aPoint == None:
                logger.error("target is occupied! Throw it!")
                continue
            wPoint = self.occuGrid.to_world_api(aPoint[0], aPoint[1])
            point = self.moveToGoal.getPoint(wPoint[0], wPoint[1]) #x,y
            ret = self.moveToPoint(point)
            if ret:
                self.topicMsg.green_image_visited.append(self.topicMsg.green_image_goals[0])
                del(self.topicMsg.green_image_goals[0])
                self.green_found += 1
                print("found green object number", self.green_found)
            else:
                # continue, time is less
                del(self.topicMsg.green_image_goals[0])

        while len(self.topicMsg.red_image_goals) > 0:
            # logger.info("Now move to red target:" + str(self.topicMsg.red_image_goals[i][0])\
            #         + "," + str(self.topicMsg.red_image_goals[i][1]))
            aPoint = self.getPointAvailable(self.topicMsg.red_image_goals[0][0], self.topicMsg.red_image_goals[0][1])
            if aPoint == None:
                logger.error("target is occupied! Throw it!")
                continue
            wPoint = self.occuGrid.to_world_api(aPoint[0], aPoint[1])
            point = self.moveToGoal.getPoint(wPoint[0], wPoint[1]) #x,y
            ret = self.moveToPoint(point)
            if ret:
                self.topicMsg.red_image_visited.append(self.topicMsg.red_image_goals[0])
                del(self.topicMsg.red_image_goals[0])
                self.red_found += 1
                print("found red object number", self.red_found)
            else:
                # continue, time is less
                del(self.topicMsg.red_image_goals[0])

    def moveToPoint(self, pose): # pose: destination
        #logger.info("Now move to: (" + str(pose.position.x) + ", " + str(pose.position.y) + ")")
        self.topicMsg.currPointToMove = pose 
        ret = self.moveToGoal.execute(pose)

        if ret:
            #logger.info("destination (" + str(pose.position.x) +\
            #        ", " + str(pose.position.y) + ") arrived!")
            pass
        else:
            #logger.error("destination (" + str(pose.position.x) +\
            #        ", " + str(pose.position.y) + ") couldn't arrived, terminate!")
            pass
        return ret

    def isObstacle(self):
        for x in self.topicMsg.forwardList:
            #logger.debug(x)
            if x < MoveParam.obstacle_dis and not x == 0:
                return True
        return False

    def isRunToTarget(self):
        if len(self.topicMsg.green_image_goals) != 0 or\
                len(self.topicMsg.red_image_goals) != 0:
                    return True
        else:
            False

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

    def findGlobalPath(self, dst):
        length = len(self.occuGrid.grid.data)
        grid_curr_x, grid_curr_y = self.occuGrid.to_grid_api(self.topicMsg.currentX, self.topicMsg.currentY)
        grid_target_x, grid_target_y = self.occuGrid.to_grid_api(dst[0],dst[1])
        #curr_x, curr_y = self.getPointAvailable(grid_curr_x, grid_curr_y)
        target_x, target_y = self.getPointAvailable(grid_target_x, grid_target_y)
        l = self.gPathFinding.aStar((grid_curr_x, grid_curr_y), (target_x, target_y))
        if l != None:
            return l
        return None

    def followPath(self):
        logger.debug("followPath entered!")
        if len(self.pathList) == 0:
            if len(self.checkpoints) == 0:
                #logger.info("points checking finished!")
                return
            else:
                dst = self.checkpoints[0]
                del(self.checkpoints[0])
                l = self.findGlobalPath(dst)

                """
                #draw the path to map
                for i in l:
                    index = self.occuGrid.to_index_api(i.x, i.y)
                    self.occuGrid.setData(index, 100)
                """

                if l == None:
                    l = []

                for i in range(len(l)): 
                    if i%25 == 0:
                        print(i)
                        self.pathList.append(l[i])
                if len(self.pathList) >= 1:
                    del(self.pathList[0])

                if len(self.pathList) != 0:
                    for p in self.pathList:
                        print(p),
                    print(" ")
                else:
                    #logger.info("gPath is none!")
                    pass

        if self.pathList != None and len(self.pathList) != 0:
            #if self.lPathFinding.status == "waitting":
                next_point = self.pathList[0]
                #delete anyway, continue to avoid blocking 
                del(self.pathList[0])
                nx, ny = self.occuGrid.to_world_api(next_point.x, next_point.y)
                point = self.moveToGoal.getPoint(nx, ny) #x,y
                self.moveToPoint(point)
                #self.lPathFinding.setTarget((next_point.x, next_point.y))
                #self.lPathFinding.setStatus("running")

    def stateCheck(self):
        self.topicMsg.lastState = self.state
        #if self.isObstacle():
        #    self.state = OBSTACLE_AVOID
        if self.isRunToTarget():
            self.state = RUN_TO_TARGET
        elif self.isRunFollowPath():
            self.state = RUN_FOLLOW_PATH
        else:
            self.state = RANDOM_WALK
    
    def isRunFollowPath(self):
        if len(self.checkpoints) != 0 or len(self.pathList) != 0:
            return True
        else:
            return False

    def logicAfterChecking(self):
        if self.topicMsg.lastState == OBSTACLE_AVOID and\
                self.state != OBSTACLE_AVOID:
            self.pub.publish(Twist(angular=Vector3(0, 0, 0)))

        if self.topicMsg.lastState != RANDOM_WALK and\
                self.state == RANDOM_WALK:
            self.topicMsg.distanceTravelled = 0

        if self.topicMsg.lastState != self.state:
            #logger.info("state: " + self.state)
            pass
         
    def pre(self):
        while rospy.Time.now().to_sec() == 0:
            time.sleep(0.1)

        while self.topicMsg.currentX is None or self.topicMsg.currentY is None:
            logger.info("current position is none")
            time.sleep(0.1)

        self.t_costMapPub.start()
        self.t_drawCostMap.start()
        #self.lPathFinding.start()
        #self.moveToChecking.start()

    def beforeStop(self):
        self.t_costMapPub.stop()
        self.t_costMapPub.join()
        self.t_drawCostMap.stop()
        self.t_drawCostMap.join()
        #self.lPathFinding.stop()
        #self.lPathFinding.join()
        #self.moveToChecking.stop()
        #self.moveToChecking.join()

    def setNewVel(self, linear, angular):
        vel = Twist(linear=Vector3(linear, 0, 0),angular=Vector3(0, 0, angular))
        self.pub.publish(vel)


    def turtleNav(self):
        self.pre()

        while not rospy.is_shutdown():  
            # every loop has 3 steps, every loop only has several milliseconds.
            # step 1
            self.stateCheck()
            # step 2
            self.logicAfterChecking()

            ############ for test, delete it######
            #self.state = RUN_FOLLOW_PATH
            #self.state = RUN_TO_TARGET

            # step 3 
            if self.topicMsg.camera_ready:

                if self.state == OBSTACLE_AVOID:
                    self.obstacleAvoid.execute()
                elif self.state == RUN_TO_TARGET:
                    self.moveToTarget()
                elif self.state == RUN_FOLLOW_PATH:
                    self.followPath()
                else:
                    self.randomWalk.execute()

                #logger.debug(self.pathMsg.dwaLinearVel)
                #logger.debug(self.pathMsg.dwaAngularVel)
                #self.setNewVel(self.pathMsg.dwaLinearVel, self.pathMsg.dwaAngularVel)

        rospy.spin()

if __name__ == '__main__':
    turtleBot = TurtleNav()
    try:
        turtleBot.turtleNav()
    except Exception as e:
        rospy.loginfo("Exception entered!")
        rospy.loginfo(e.args)
        rospy.loginfo(traceback.format_exc())
        turtleBot.beforeStop()
