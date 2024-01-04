import rospy
import time
import threading
import math
import tf
from math import cos, sin
from common.DataStruct import TopicMsg
from geometry_msgs.msg import Twist, Point, PointStamped
from Logger import logger

class DrawCostMap(threading.Thread):
    def __init__(self, topicMsg, occupancyGrid):
        threading.Thread.__init__(self)
        self.topicMsg = topicMsg
        self.occuGrid = occupancyGrid
        self.isStop = False
        self.listener = tf.TransformListener()

    def stop(self):
        self.isStop = True

    def run(self):
        while not self.isStop:
            if self.topicMsg.scan_msg == None or\
                    self.topicMsg.odom_msg == None:
                continue
            if self.topicMsg.currentX == None or\
                    self.topicMsg.currentY == None:
                continue

            odom_msg = self.topicMsg.odom_msg
            scan_msg = self.topicMsg.scan_msg
            quarternion = [odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,\
                        odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
            currentTheta = yaw
            currentX = odom_msg.pose.pose.position.x
            currentY = odom_msg.pose.pose.position.y

            try:
                ### Need to run Rviz to work
                self.listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(4.0))
                #trans, rot = self.listener.lookupTransform('map', 'odom', rospy.Time(0))
                #matrix = numpy.asmatrix(self.listener.lookupTransform('map', 'odom', rospy.Time(0))) 
                point = Point()
                point.x = 0.0
                point.y = 0.0
                point.z = 0.0
                point_tf = PointStamped()
                point_tf.header.frame_id = "base_link"
                point_tf.header.stamp = rospy.Time(0)
                point_tf.point = point

                map_p = self.listener.transformPoint("map", point_tf)
                currentX = map_p.point.x
                currentY = map_p.point.y
                gx,gy = self.occuGrid.to_grid_api(currentX, currentY)
                #print("bot=", gx, gy)

            #print(map_p.point)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            #self.updateTrace(odom_msg)
            #self.updateEmptySpace(scan_msg, currentTheta, currentX, currentY)
            self.updateObstacle(scan_msg, currentTheta, currentX, currentY)
            #self.topicMsg.grid = self.occuGrid.grid
            time.sleep(0.1) # s

    def updateTrace(self, msg):
        # it will be dangerous when resolution bigger than 0.05 
        gx, gy = self.occuGrid.to_grid_api(msg.pose.pose.position.x, msg.pose.pose.position.y)
        i = self.occuGrid.to_index_api(gx, gy)
        self.occuGrid.setData(i, 0.05)

    def updateEmptySpace(self, msg, currentTheta, currentX, currentY):
        laser_min = msg.range_min
        laser_max = msg.range_max
        single_max = None
        
        for i in range(0, 360):
            dis = msg.ranges[i]
            ang = i

            """
            if dis == float('inf'):
                single_max = laser_max
            else:
                single_max = dis
                """
            if dis == float('inf'):
                continue

            single_max = dis
            
            self.bayesianEmptySpace(msg, single_max, i, currentTheta, currentX, currentY)
            
    
    def bayesianEmptySpace(self, msg, dis, ang, currentTheta, currentX, currentY):
        gx, gy = self.occuGrid.to_grid_api(currentX, currentY)
        lx, ly = self.occuGrid.getGridPosOfPoint(dis, ang, currentTheta, currentX, currentY)
        #print("gx = " , gx , ",gy = " , gy , " - lx = " , lx , ",ly = " , ly )
        points = self.getLine((gx, gy), (lx, ly))
        # print(points, "\n")
        for p in points:
            max_r = msg.range_max
            dx = abs(p[0] - gx)
            dy = abs(p[1] - gy)
            r = math.sqrt((dx*dx) + (dy*dy))
            prob = abs(((max_r - r) / max_r) / 2)
            # print(prob)
            #prob = 0.05
            i = self.occuGrid.to_index_api(p[0], p[1])
            # if p == points[-1]:
            #     self.occuGrid.setData(i, 100)
            if self.occuGrid.getData()[i] == -1:
                self.occuGrid.setData(i, prob)
            elif prob < self.occuGrid.getData()[i] and self.occuGrid.getData()[i] != 100:
                self.occuGrid.setData(i, prob)


    def updateObstacle(self, msg, currentTheta, currentX, currentY):
        # anti the clock sequence, 12-> 9->6->3->12
        # 0-359
        for i in range(0, 360):
            dis = msg.ranges[i]
            ang = i

            if dis == float('inf') or currentTheta == None:
                logger.debug("inf value, pass!")
                continue

            index = self.occuGrid.getIndexOfPoint(dis, ang, currentTheta, currentX, currentY)
            self.occuGrid.setData(index,100)
            logger.debug(index)

    def getLine(self, start, end):
        ####
        #### Using Bresenhams Line Drawing Algorithm given in minitask4
        ####

        #print("start = ", start, "end = ", end)
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
    
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
    
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
    
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
    
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()

        if points[-1] != end:
            print("start = ", start, "end = ", end)
            print("alert")
            print(points[-1])
            print("===========")

        #print(points)
        
        return points



