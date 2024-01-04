#!/usr/bin/env python

import sys
sys.path.append("..")
import unittest
from common.OccupiedGrid import OccupiedGrid

class indexTests(unittest.TestCase):

    def test1(self):
        grid = OccupiedGrid()
        (gx, gy) = (5, 5)
        size_x = 20
        testx, testy = OccupiedGrid.from_index(grid, grid.to_index(gx, gy, size_x), size_x)
        self.assertEqual((gx, gy), (testx, testy))

    def testIndex1(self): 
        grid = OccupiedGrid()
        grid_x = 0
        grid_y = 0
        index = grid.to_index_api(grid_x, grid_y)
        print("grid_x:" + str(grid_x) + " grid_y:" + str(grid_y) +\
                " index: " + str(index))

    def testIndex2(self): 
        grid = OccupiedGrid()
        grid_x = 1
        grid_y = 0
        index = grid.to_index_api(grid_x, grid_y)
        print("grid_x:" + str(grid_x) + " grid_y:" + str(grid_y) +\
                " index: " + str(index))

        grid.setData(index, 5)
        grid.drawCostMap()

    def testIndex3(self): 
        grid = OccupiedGrid()
        grid_x = 0
        grid_y = 1
        index = grid.to_index_api(grid_x, grid_y)
        print("grid_x:" + str(grid_x) + " grid_y:" + str(grid_y) +\
                " index: " + str(index))

        grid.setData(index, 5)
        grid.drawCostMap()

    def testIndex4(self): 
        grid = OccupiedGrid()
        grid_x = 1
        grid_y = 1
        index = grid.to_index_api(grid_x, grid_y)

        grid.setData(index, 5)
        grid.printByIndex(index)
        print("grid_x:" + str(grid_x) + " grid_y:" + str(grid_y) +\
                " index: " + str(index))

        grid.drawCostMap()

if __name__ == '__main__':
    unittest.main()
