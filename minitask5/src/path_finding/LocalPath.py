from Logger import logger
from common.DataStruct import PathMsg
from common.DataStruct import TopicMsg
import time
import threading
from Dwa import dwa_core,Info, motion_model
import numpy as np
import math

class LocalPath(threading.Thread):
    def __init__(self, topicMsg, pathMsg, occuGrid):
        threading.Thread.__init__(self)
        self.pathMsg = pathMsg
        self.topicMsg = topicMsg
        self.occuGrid = occuGrid
        self.isStop = False
        self.currVelocity = None #should be zero
        self.target = None
        self.status = "waitting" # running
    
    def setTarget(self, target):
        self.target = target
        logger.info("new dst:[" + str(target[0]) + "," + str(target[1]) + "]")

    def setStatus(self, status):
        self.status = status
    def getVel(self):
        return self.currVelocity

    def stop(self):
        self.isStop = True

    def run(self):
        while not self.isStop:
            if self.target == None:
                time.sleep(0.1) # s
                continue

            if self.status == "waitting":
                time.sleep(0.1) # s
                continue

            if self.pathMsg.lPathModel == "dwa":
                # x, y, theta,linear speed, angular speed 
                grid_curr_x, grid_curr_y = self.occuGrid.to_grid_api(\
                        self.topicMsg.currentX, self.topicMsg.currentY)
                x = np.array([grid_curr_x, grid_curr_y,self.topicMsg.currentTheta, 0, 0])
                # init speed
                u = np.array([0,0])
                # target location
                goal = np.array([self.target[0],self.target[1]])
                info = Info()
                obstacles = np.array(self.occuGrid.grid.data)\
                        .reshape(self.occuGrid.grid.info.width, self.occuGrid.grid.info.height)
                # trajction
                global_traj = np.array(x)

                self.dwa(u,x,info,goal,obstacles)

            time.sleep(0.1) # s

    def dwa(self,u,x,info,goal,obstacles):
        while(self.status == "running"):
            u,current_traj = dwa_core(x,u,goal,info,obstacles)
            self.pathMsg.dwaLinearVel = u[0] #  linear speed
            self.pathMsg.dwaAngularVel = u[1] #  angular speed
            grid_curr_x, grid_curr_y = self.occuGrid.to_grid_api(self.topicMsg.currentX, self.topicMsg.currentY)
            x = motion_model(x,u,info.dt)
            x[0] = grid_curr_x
            x[1] = grid_curr_y
            logger.info("dwa vel:" + str(u[0]) + "," + str(u[1]))
            #best trajction
            #global_traj = np.vstack((global_traj,x))

            # judge if reach goal location
            if math.sqrt((x[0]-goal[0])**2 + (x[1]-goal[1])**2) <= info.radius:
                    print ("Goal Arrived!")
                    self.status = "waitting"

            time.sleep(0.1) # s
