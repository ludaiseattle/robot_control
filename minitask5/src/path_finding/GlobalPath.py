import time
import threading
from Logger import logger
from pylab import *
from common.DataStruct import PathMsg
from AStar import AStar, Point, Array2D
from common.OccupiedGrid import OccupiedGrid

class GlobalPath():
    def __init__(self, pathMsg, occuGrid):
        self.pathMsg = pathMsg
        self.occuGrid = occuGrid
        self.startP = None
        self.endP = None
        self.map = None
    
    def aStar(self, start, end):
        logger.info("global path finding:" +\
                " start(" + str(start[0])+ "," + str(start[1])+\
                ") end(" + str(end[0])+ "," + str(end[1])+")")
        self.map = Array2D(self.occuGrid.grid.info.width, self.occuGrid.grid.info.height, list(self.occuGrid.grid.data))     
        #self.map.showArray2D()
        if self.map == None:
            logger.info("map is null! Waitting...")
            return
        aStar=AStar(self.map, Point(start[0], start[1]), Point(end[0], end[1]))
        pathList = aStar.start()
        return pathList

