#!/usr/bin/env python

import sys
import math
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
sys.path.append("..")
import unittest
from common.OccupiedGrid import OccupiedGrid
from math import cos, sin

class robotToMapTests(unittest.TestCase):

    def test10(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = 3
        dis = 2

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test11(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = 0 
        dis = 0

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test12(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = math.pi
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test13(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = 0
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test14(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = math.pi/12  #30 degree
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test15(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = math.pi/6  #60 degree
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test15(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = math.pi/4  #90 degree
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test16(self):
        """distance and angle to xy """
        grid = OccupiedGrid()

        # radar data
        ang = (math.pi/6)*5  #-30 degree
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " laser_x: " + str(laser_x) + " laser_y: " + str(laser_y))

    def test20(self):
        """ radar to world"""
        grid = OccupiedGrid()

        # radar data
        ang = 0
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)

        #robot msg 
        currentTheta = 0
        currentX = 0 
        currentY = 0 

        world_x = cos(currentTheta) * laser_x - sin(currentTheta)* laser_y + currentX
        world_y = sin(currentTheta) * laser_x + cos(currentTheta)* laser_y + currentY
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " currentTheta: " + str(currentTheta) +\
                " currentX: " + str(currentX) + " currentY: " + str(currentY) +\
                " world_x: " + str(world_x) + " world_y: " + str(world_y))

    def test21(self):
        """ radar to world"""
        grid = OccupiedGrid()

        # radar data
        ang = math.pi/4
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)

        #robot msg 
        currentTheta = 0
        currentX = 0 
        currentY = 0 

        world_x = cos(currentTheta) * laser_x - sin(currentTheta)* laser_y + currentX
        world_y = sin(currentTheta) * laser_x + cos(currentTheta)* laser_y + currentY
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " currentTheta: " + str(currentTheta) +\
                " currentX: " + str(currentX) + " currentY: " + str(currentY) +\
                " world_x: " + str(world_x) + " world_y: " + str(world_y))

    def test22(self):
        """ radar to world"""
        grid = OccupiedGrid()

        # radar data
        ang = math.pi/4
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)

        #robot msg 
        currentTheta = math.pi/4
        currentX = 0 
        currentY = 0 

        world_x = cos(currentTheta) * laser_x - sin(currentTheta)* laser_y + currentX
        world_y = sin(currentTheta) * laser_x + cos(currentTheta)* laser_y + currentY
        print("ang: " + str(ang) + " dis: " + str(dis) +\
                " currentTheta: " + str(currentTheta) +\
                " currentX: " + str(currentX) + " currentY: " + str(currentY) +\
                " world_x: " + str(world_x) + " world_y: " + str(world_y))

    def test3(self):
        """ world to grid"""
        grid = OccupiedGrid()

        # radar data
        ang = math.pi/4
        dis = 1

        laser_x = dis * cos(ang) 
        laser_y = dis * sin(ang)

        #robot msg 
        currentTheta = 0
        currentX = 0 
        currentY = 0 


        world_x = cos(currentTheta) * laser_x - sin(currentTheta)* laser_y + currentX
        world_y = sin(currentTheta) * laser_x + cos(currentTheta)* laser_y + currentY
        grid_x, grid_y = grid.to_grid_api(world_x, world_y)
        index = grid.to_index_api(grid_x, grid_y)
        grid.setData(index, 5)

        grid.drawCostMap()


if __name__ == '__main__':
    unittest.main()
