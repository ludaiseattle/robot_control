"""
just use it as an exists model

"""
from Logger import logger
import time

class Array2D:
    def __init__(self,w,h, data):
        self.w=w
        self.h=h
        self.data=data
        #self.data=[[0 for y in range(h)] for x in range(w)]

    def showArray2D(self):
        for y in range(self.h):
            for x in range(self.w):
                print(". " + str(x) + "," + str(y) + " " + str(self.data[y*self.w+x])),
            print("")

    def __getitem__(self, item):
        return self.data[item]


class Point:
    def __init__(self,x,y):
        self.x=x;self.y=y
 
    def __eq__(self, other):
        if self.x==other.x and self.y==other.y:
            return True
        return False
    def __str__(self):
        return "x:"+str(self.x)+",y:"+str(self.y)

class AStar:
    class Node:  
        def __init__(self, point, endPoint, g=0):
            self.point = point  
            self.father = None  
            self.g = g  
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y)) * 15 
 
    def __init__(self, map2d, startPoint, endPoint, passTag=0):
        self.openList = []
        self.closeList = []
        self.map2d = map2d
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)
 
        self.passTag = passTag
 
    def getMinNode(self):
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode
 
    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point == point:
                return True
        return False
 
    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None
 
    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None
 
    def searchNear(self, minF, offsetX, offsetY):
        if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.map2d.w - 1 or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.map2d.h - 1:
            return
        if self.map2d[(minF.point.y+offsetY)*self.map2d.w+(minF.point.x + offsetX)] != self.passTag:
            return
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)
        if self.pointInCloseList(currentPoint):
            return
        if offsetX == 0 or offsetY == 0:
            step = 10
        else:
            step = 14
        currentNode = self.pointInOpenList(currentPoint)
        if not currentNode:
            currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step)
            currentNode.father = minF
            self.openList.append(currentNode)
            return
        if minF.g + step < currentNode.g:  
            currentNode.g = minF.g + step
            currentNode.father = minF
 
    def start(self):
        if self.map2d[self.endPoint.y*self.map2d.w+self.endPoint.x] != self.passTag:
            logger.info("location is occupied")
            logger.info(self.map2d[self.endPoint.y*self.map2d.w+self.endPoint.x])
            return None
 
        startNode = AStar.Node(self.startPoint, self.endPoint)
        start = time.time()
        self.openList.append(startNode)
        while True:
            minF = self.getMinNode()
            self.closeList.append(minF)
            self.openList.remove(minF)
            span = 1
            self.searchNear(minF, 0, -span)
            self.searchNear(minF, 0, span)
            self.searchNear(minF, -span, 0)
            self.searchNear(minF, span, 0)
            point = self.endPointInCloseList()
            if point:  
                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        # print(pathList)
                        # print(list(reversed(pathList)))
                        # print(pathList.reverse())
                        last = time.time()
                        logger.info("astar cost:" + str(last-start))
                        return list(reversed(pathList))
            if len(self.openList) == 0:
                return None
