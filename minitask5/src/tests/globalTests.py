#!/usr/bin/env python

import sys
sys.path.append("..")
import unittest
from PIL import Image
from pylab import *
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from path_finding.AStar import *

class globalTests(unittest.TestCase):
    def test1(self):
        #read map
        im = array(Image.open("../../maps/train_env.pgm").convert('1'),'f')
        #print(im.shape, im.dtype)
        #print(im[100,100])
        for i in range(0, len(im)):
            for j in range(0, len(im[i])):
                im[i,j] = -im[i,j]+1
                print(i,j, ":", im[i,j])

        map2d=Array2D(im.shape[0],im.shape[1], im)
        #map2d.showArray2D()
        aStar=AStar(map2d,Point(0,0),Point(100,100))
        pathList=aStar.start()
        for point in pathList:
            map2d[point.x][point.y]=8
            print(point)
        print("----------------------")
        #map2d.showArray2D()

if __name__ == '__main__':
    unittest.main()
