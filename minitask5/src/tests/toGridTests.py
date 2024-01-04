#!/usr/bin/env python

import sys
sys.path.append("..")
import unittest
from common.OccupiedGrid import OccupiedGrid
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion

class toGridTests(unittest.TestCase):

    def testValid1(self):
        grid = OccupiedGrid()
        testx, testy = grid.to_grid(5, 5, Pose(Point(0,0,0),Quaternion(0,0,0,0.0)), (20,20), 1)
        self.assertEqual((testx, testy), (5, 5))

    def testValid2(self):
        grid = OccupiedGrid()
        testx, testy = grid.to_grid(5, 5, Pose(Point(0,0,0),Quaternion(0,0,0,0.0)), (20,20), 0.5)
        self.assertEqual((testx, testy), (10, 10))

    def testValid3(self):
        grid = OccupiedGrid()
        testx, testy = grid.to_grid(0, 0, Pose(Point(-5,-5,0),Quaternion(0,0,0,0.0)), (20,20), 1)
        self.assertEqual((testx, testy), (5, 5))

    def testInverse(self):
        grid = OccupiedGrid()
        px, py = grid.to_world(9, 9, Pose(Point(-5,-5,0),Quaternion(0,0,0,0.0)), (20,20), 1.0)
        testx, testy = grid.to_grid(px, py, Pose(Point(-5,-5,0),Quaternion(0,0,0,0.0)), (20, 20), 1.0)
        self.assertEqual((testx, testy), (9, 9))

    def testToGrid1(self): 
        grid = OccupiedGrid()
        world_x = 0
        world_y = 0
        grid_x, grid_y = grid.to_grid_api(world_x, world_y)
        print("world_x:" + str(world_x) + " world_y:" + str(world_y) +\
                " grid_x:" + str(grid_x) + " grid_y:" + str(grid_y))

    def testToGrid2(self): 
        grid = OccupiedGrid()
        world_x = 1
        world_y = 1
        grid_x, grid_y = grid.to_grid_api(world_x, world_y)
        print("world_x:" + str(world_x) + " world_y:" + str(world_y) +\
                " grid_x:" + str(grid_x) + " grid_y:" + str(grid_y))
        #index = grid.to_index_api(grid_x, grid_y)
        #print("index: " + str(index))
        #grid.setData(index, 5)

        #grid.drawCostMap()

if __name__ == '__main__':
    unittest.main()
