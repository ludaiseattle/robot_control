import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from Global import *
from Logger import logger
from math import cos, sin
from common import MoveParam
from common.DataStruct import TopicMsg
from common import OccupiedGridParams as params
import math

class OccupiedGrid():
    def __init__(self):
        self.grid = OccupancyGrid()
        self.grid.header.seq = 0
        self.grid.info.resolution = params.resolution
        self.grid.info.width = params.width
        self.grid.info.height = params.height
        self.grid.info.origin = params.origin 
        self.occupied_thresh = params.occupied_thresh
        self.free_thresh = params.free_thresh
        self.size_x = self.grid.info.width
        self.size_y = self.grid.info.height
        self.topicMsg = None

        self.grid.data = [-1 for i in range(self.size_x * self.size_y)]
        #self.drawCostMap()

    def setTopicMsg(self, topicMsg):
        self.topicMsg = topicMsg

    def to_grid_api(self, px, py):
        origin = self.grid.info.origin
        size = (self.size_x, self.size_y)
        return self.to_grid(px, py, origin, size, self.grid.info.resolution)
        
    def to_grid(self, px, py, origin_, size, resolution):
        """
         gx gy: index of cell
         size: total number of cells in length or width
        """ 
        gx = int((px/resolution) - (origin_.position.x/resolution))
        gy = int((py/resolution) - (origin_.position.y/resolution))
        # origin could be negative
        ## ANSWER: looking at grid positions therefore gx never negative
        ## as whilst origin of grid may have world coordinates of e.g. (-5, -5)
        ## the grid coordinate is (0, 0)

        if gx > size[0]:    
            raise ValueError("Invalid x: outside grid - TOO LARGE")
        elif gx < 0:
            raise ValueError("Invalid x: outside grid - LESS THAN ZERO")

        if gy > size[1]:
            raise ValueError("Invalid y: outside grid - TOO LARGE")
        elif gy < 0:
            raise ValueError("Invalid y: outside grid - LESS THAN ZERO")

        return gx, gy

    def to_world_api(self, gx, gy): 
        origin = self.grid.info.origin
        size = (self.size_x, self.size_y)
        return self.to_world(gx, gy, origin, size, self.grid.info.resolution)

    def to_world(self, gx, gy, origin, size, resolution):
        px = (gx * resolution) + origin.position.x + resolution/2
        py = (gy * resolution) + origin.position.x + resolution/2
        logger.info("gx: " + str(gx) + " gy: " + str(gy) +\
                " px: " + str(px) + " py: " + str(py))

        return px, py

    def to_index_api(self, gx, gy):
        return self.to_index(gx, gy, self.size_x)

    def to_index(self, gx, gy, size_x):
        """
        index is the index of cell, cell is a square in resolution
        """
        index = gy * size_x + gx 
        logger.debug("gx: " + str(gx) + " gy: " +\
                str(gy) + " index: " + str(index))
        return index

    def from_index(self, i, size_x):
        gx = i%size_x
        gy = i//size_x

        return gx, gy

    def from_index_api(self, i, size_x):
        return self.from_index(i, self.size_x)
        
    ## Can be used to access the grid data
    def getData(self):
        return self.grid.data

    def setData(self, index, value):
        logger.debug("write data:  index: " + str(index) + " value: " + str(value))
        self.grid.data[index] = value

    def printByIndex(self, index):
        print("index:" + str(index) + " value:" + str(self.grid.data[index]))

    def getIndexOfPoint(self, dis, ang, currentTheta, currentX, currentY):
        grid_x, grid_y = self.getGridPosOfPoint(dis, ang, currentTheta, currentX, currentY)
        index = self.to_index_api(grid_x, grid_y)

        return index

    def getGridPosOfPoint(self, dis, ang, currentTheta, currentX, currentY):
        laser_x = dis * cos(math.radians(ang)) 
        laser_y = dis * sin(math.radians(ang))
        world_x = cos(currentTheta) * laser_x - sin(currentTheta)* laser_y + currentX
        world_y = sin(currentTheta) * laser_x + cos(currentTheta)* laser_y + currentY
        logger.debug("angle: " + str(ang) + " world_x: " + str(world_x) + " world_y: " +  str(world_y))
        grid_x, grid_y = self.to_grid_api(world_x, world_y)

        return grid_x, grid_y

    def drawCostMap(self):
        fo = open("cost_map.txt", "w")
        for i in range(self.size_y):
            for j in range(self.size_x):
                value =self.grid.data[i*self.size_x + j] 
                if value == -1:
                    fo.write("U")
                elif value >= 0 and value <= 3:
                    fo.write("Y")
                else:
                    fo.write("N")
            fo.write("\n")

        fo.write("\n")
        for i in range(self.size_y):
            for j in range(self.size_x):
                value = self.grid.data[i*self.size_x + j] 
                fo.write(str(i*self.size_x + j) +  ":" + str(value) + " ")
            fo.write("\n")
        fo.close()

    def drawMap(self):
        self.grid.header.seq += 1
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "odom"

