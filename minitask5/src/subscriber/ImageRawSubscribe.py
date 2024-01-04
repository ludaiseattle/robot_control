import rospy
import tf, tf2_ros, tf2_py
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import OccupancyGrid
from common.OccupiedGrid import OccupiedGrid
import cv2, cv_bridge
from cv2 import blur
import numpy 
import message_filters
import math
from Logger import logger
from geometry_msgs.msg import Twist, Point, PointStamped
from common import MoveParam
from Global import *

class ImageRawSubscribe():
    def __init__(self, rospy, topicMsg, occuGrid):
        self.printimg = True
        self.rospy = rospy
        self.occuGrid = occuGrid
        self.topicMsg = topicMsg
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        #self.image_sub = self.rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        #self.depth_sub = self.rospy.Subscriber("/camera/depth/points", PointCloud2, self.depth_callback)
        self.image_sub = message_filters.Subscriber("camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/points", PointCloud2)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.image_callback)
        self.result = None
        self.stopCheck = None
        self.hit = None
        self.pixelRange = 50
        self.listener = tf.TransformListener()
        self.pointpub = rospy.Publisher('point_msgs', PointStamped, queue_size=1)

    def image_callback(self, image_msg, depth_msg):
          try:
            self.listener.waitForTransform("map", "camera_depth_optical_frame", image_msg.header.stamp, rospy.Duration(4.0))
          except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
            print("exit beginning img callback")
            self.topicMsg.camera_ready = False
            return

          self.topicMsg.camera_ready = True

          logger.debug("entered!")
          self.result = None
          image = self.bridge.imgmsg_to_cv2(image_msg,desired_encoding='bgr8')
          (h, w) = image.shape[:2]
          image_resized = cv2.resize(image, (w/4,h/4))
          self.result = image_resized.copy()
          hsv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
          depth_data = point_cloud2.read_points(depth_msg, field_names = ("x", "y", "z"), skip_nans=True)


          # for p in depth_data:
          #   print(p)

          # Adding hsv ranges for each of the colours in the task
          #GREEN
          green_lower = numpy.array([40, 140, 20])
          green_upper = numpy.array([70, 255, 255])

          #RED red wraps around
          red_lower1 = numpy.array([0, 220, 5])
          red_upper1 = numpy.array([10, 255, 45])
          red_lower2 = numpy.array([170, 220, 5])
          red_upper2 = numpy.array([180, 255, 45])

          #BLUE
          blue_lower = numpy.array([110,160,20])
          blue_upper = numpy.array([130,255,255])

          ## TODO: add contour, try get depth working

          #Creating colour masks
          green_mask = cv2.inRange(hsv_image, green_lower, green_upper)

          red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
          red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)

          avoid_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
          
          red_mask = red_mask1 | red_mask2

          #Removing noise from the masks by erosion, gets rid of small individual pixels in mask
          kernel = numpy.ones((2, 2), numpy.uint8)

          red_mask = cv2.erode(red_mask, kernel, iterations=1)
          green_mask = cv2.erode(green_mask, kernel, iterations=1)
          avoid_mask = cv2.erode(avoid_mask, kernel, iterations=1)

          mask = green_mask | red_mask1 | red_mask2

          #Creating contours for each of the colours
          _, green_contours, _ = cv2.findContours(image=green_mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
          _, red_contours, _ = cv2.findContours(image=red_mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
          _, blue_contours, _ = cv2.findContours(image=avoid_mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

          #print(len(green_contours))
          green_res = image_resized.copy()
          green_res = cv2.bitwise_and(green_res, green_res, mask=green_mask)


          #creating bounding boxes around the shape using contours
          for i in green_contours:
            x,y,w,h = cv2.boundingRect(i)
            
            
            for g in range(x, x + w):
              cv2.rectangle(img=image_resized, pt1=(x,y),pt2=(x+w,y+h), color=(0,255,0),thickness=2, lineType=cv2.LINE_AA)
              cv2.circle(img=image_resized, center=(g, y+(h/2)), radius=5, color=(0,255,0), thickness=-1)
              cv2.rectangle(img=green_res, pt1=(x,y),pt2=(x+w,y+h), color=(0,255,0),thickness=2, lineType=cv2.LINE_AA)
              cv2.circle(img=green_res, center=(g, y+(h/2)), radius=5, color=(0,255,0), thickness=-1)
              #print("check1")
              t = green_mask[(y+(h/2))][g]
              #print("t=", t)
              #print("g=", g, "y=", (y+(h/2)))
              if t != 0:
                #print("check2")
                pos = point_cloud2.read_points(depth_msg, field_names = ("x", "y", "z"), skip_nans=True, uvs=[(g*4, (y+(h/2))*4)])
                #check = point_cloud2.read_points_list(depth_msg, skip_nans=True, uvs=[(x+(w/2), y+(h/2))])
                #print(check)
                point = Point()
                for p in pos:
                      point.x = p[0]
                      point.y = p[1]
                      point.z = p[2]

                try:
                    ### Need to run Rviz to work
                    

                    point_tf_msg = PointStamped()
                    point_tf_msg.header.frame_id = "camera_depth_optical_frame"
                    point_tf_msg.header.stamp = image_msg.header.stamp
                    point_tf_msg.point = point
                    #self.pointpub.publish(point_tf_msg)
                    map_p = self.listener.transformPoint("map", point_tf_msg)
                    self.pointpub.publish(map_p)

                    ### DRAW IT ON THE MAP

                    gx, gy = self.occuGrid.to_grid_api(map_p.point.x, map_p.point.y)
                    index = self.occuGrid.to_index_api(gx, gy)
                    self.occuGrid.setData(index, 100)

                    
                    visited = False
                    for cx in range(gx+30, gx+31):
                      for cy in range(gy-30, gy+31):
                        if (cx, cy) in self.topicMsg.green_image_visited:
                          visited = True
                    
                    seen = False
                    if not visited:
                      for cx in range(gx-30, gx+31):
                        for cy in range(gy-30, gy+31):
                          if (cx, cy) in self.topicMsg.green_image_goals:
                            seen = True
                    
                    if not seen:
                      self.topicMsg.green_image_goals.append((gx, gy))

                    #print("green=", self.topicMsg.green_image_goals)


                    #print(map_p.point)
                    #print("obstacle=", gx, gy)
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                  #print("exception")
                  pass

          #cv2.imshow('green', green_res)

          for i in red_contours:
            x,y,w,h = cv2.boundingRect(i)
            for g in range(x, x + w):
              cv2.rectangle(img=image_resized, pt1=(x,y),pt2=(x+w,y+h), color=(0,0,255),thickness=2, lineType=cv2.LINE_AA)
              cv2.circle(img=image_resized, center=(g, y+(h/2)), radius=5, color=(0,0,255), thickness=-1)
              #cv2.rectangle(img=green_res, pt1=(x,y),pt2=(x+w,y+h), color=(0,255,0),thickness=2, lineType=cv2.LINE_AA)
              #cv2.circle(img=green_res, center=(g, y+(h/2)), radius=5, color=(0,255,0), thickness=-1)
              #print("check1")
              t = red_mask[(y+(h/2))][g]
              #print("t=", t)
              #print("g=", g, "y=", (y+(h/2)))
              if t != 0:
                #print("check2")
                pos = point_cloud2.read_points(depth_msg, field_names = ("x", "y", "z"), skip_nans=True, uvs=[(g*4, (y+(h/2))*4)])
                #check = point_cloud2.read_points_list(depth_msg, skip_nans=True, uvs=[(x+(w/2), y+(h/2))])
                #print(check)
                point = Point()
                for p in pos:
                      point.x = p[0]
                      point.y = p[1]
                      point.z = p[2]

                try:
                    ### Need to run Rviz to work
                    
                    point_tf_msg = PointStamped()
                    point_tf_msg.header.frame_id = "camera_depth_optical_frame"
                    point_tf_msg.header.stamp = image_msg.header.stamp
                    point_tf_msg.point = point
                    #self.pointpub.publish(point_tf_msg)
                    map_p = self.listener.transformPoint("map", point_tf_msg)
                    self.pointpub.publish(map_p)

                    ### DRAW IT ON THE MAP

                    gx, gy = self.occuGrid.to_grid_api(map_p.point.x, map_p.point.y)
                    index = self.occuGrid.to_index_api(gx, gy)
                    self.occuGrid.setData(index, 50)

                    visited = False
                    for cx in range(gx+30, gx+31):
                      for cy in range(gy-30, gy+31):
                        if (cx, cy) in self.topicMsg.red_image_visited:
                          visited = True
                    
                    seen = False
                    if not visited:
                      for cx in range(gx-30, gx+31):
                        for cy in range(gy-30, gy+31):
                          if (cx, cy) in self.topicMsg.red_image_goals:
                            seen = True
                    
                    if not seen:
                      self.topicMsg.red_image_goals.append((gx, gy))

                    #print("red=", self.topicMsg.red_image_goals)

                    #print(map_p.point)
                    #print("obstacle=", gx, gy)
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                  #print("exception")
                  pass
          
          for i in blue_contours:
            x,y,w,h = cv2.boundingRect(i)
            for g in range(x, x + w):
              cv2.rectangle(img=image_resized, pt1=(x,y),pt2=(x+w,y+h), color=(255,0,0),thickness=2, lineType=cv2.LINE_AA)
              cv2.circle(img=image_resized, center=(g, y+(h/2)), radius=5, color=(255,0,0), thickness=-1)
              #cv2.rectangle(img=green_res, pt1=(x,y),pt2=(x+w,y+h), color=(0,255,0),thickness=2, lineType=cv2.LINE_AA)
              #cv2.circle(img=green_res, center=(g, y+(h/2)), radius=5, color=(0,255,0), thickness=-1)
              #print("check1")
              t = avoid_mask[(y+(h/2))][g]
              #print("t=", t)
              #print("g=", g, "y=", (y+(h/2)))
              if t != 0:
                #print("check2")
                pos = point_cloud2.read_points(depth_msg, field_names = ("x", "y", "z"), skip_nans=True, uvs=[(g*4, (y+(h/2))*4)])
                #check = point_cloud2.read_points_list(depth_msg, skip_nans=True, uvs=[(x+(w/2), y+(h/2))])
                #print(check)
                point = Point()
                for p in pos:
                      point.x = p[0]
                      point.y = p[1]
                      point.z = p[2]

                try:
                    ### Need to run Rviz to work
                    
                    point_tf_msg = PointStamped()
                    point_tf_msg.header.frame_id = "camera_depth_optical_frame"
                    point_tf_msg.header.stamp = image_msg.header.stamp
                    point_tf_msg.point = point
                    #self.pointpub.publish(point_tf_msg)
                    map_p = self.listener.transformPoint("map", point_tf_msg)
                    self.pointpub.publish(map_p)

                    ### DRAW IT ON THE MAP

                    gx, gy = self.occuGrid.to_grid_api(map_p.point.x, map_p.point.y)
                    index = self.occuGrid.to_index_api(gx, gy)
                    if self.occuGrid.grid.data[index] != 0:
                        self.topicMsg.blue_seen = True
                    self.occuGrid.setData(index, 100)

                    #print(map_p.point)
                    #print("obstacle=", gx, gy)
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                  #print("exception")
                  pass


          #cv2.drawContours(image=image_resized, contours=green_contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
          #cv2.drawContours(image=image_resized, contours=red_contours, contourIdx=-1, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_AA)
          #cv2.drawContours(image=image_resized, contours=blue_contours, contourIdx=-1, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)

          self.result = cv2.bitwise_and(self.result, self.result, mask=mask)

          #cv2.imshow('mask', avoid_mask)
          #cv2.imshow('green_mask', green_mask)
          #cv2.imshow('result', self.result)
          
          cv2.imshow("original", image_resized)
          #logger.info("after")
          # cv2.imshow("hsv", hsv_image)
          cv2.waitKey(3)
          
          h, w = self.result.shape[:2]
          rows,cols,_ = self.result.shape
          if self.hit != None:
            check = self.result[self.hit[1], self.hit[0]]
            if check[2] != 0:
              pass
            else:
              self.hit = None

          # if self.hit == None:
          #   for y in range(rows):
          #     k = self.result[y,w/2]
          #     if k[2] != 0:
          #               logger.debug("location of green: i-" + str(y) + " j-" + str(w/2))
          #               self.hit = (w/2,y)
          #               #logger.info("move_to cancelling!")
          #               if self.topicMsg.move_base != None:
          #                   pass
          #                   #self.topicMsg.move_base.cancel_goal()
          #               break

          # if self.hit == None:
          #     logger.debug("get new green hit!")
          #     for i in range(rows):
          #       for j in range(cols):
          #           k = self.result[i,j]
          #           if k[2] != 0:
          #               logger.debug("location of green: i-" + str(i) + " j-" + str(j))
          #               self.hit = (j,i)
          #               break

          self.stopCheck = self.result.copy()

          s_h, s_w = self.stopCheck.shape[:2]
          s_rows,s_cols,_ = self.stopCheck.shape

          search_top = 7*h/8
          
          self.stopCheck[0:search_top, 0:s_w] = 0
          # self.stopCheck[search_bot:s_h, 0:s_w] = 0

          for a in range(s_rows):
                for b in range(s_cols):
                    k = self.stopCheck[a,b]
                    if k[2] != 0:
                        # logger.debug("Found Green")
                        MoveParam.found_green = 1
                        break                    

          """
          if self.hit == None:
                self.topicMsg.greenVel = Twist()
                self.topicMsg.toGreenDirection = "nothing"
                logger.debug("direction: " + self.topicMsg.toGreenDirection)
                logger.debug("exit!")
                return
          else:
              diff = self.hit[0] - w/2
              self.topicMsg.greenVel.linear.x = MoveParam.linear_speed
              self.topicMsg.greenVel.angular.z = -float(diff) / MoveParam.p_divider

              # print(w, h)
              # print(self.hit[1])
              # print("diff:" , diff)
              # print("lin:" , self.topicMsg.greenVel.linear.x , "ang:" , self.topicMsg.greenVel.angular.z)

              if abs(self.hit[1] - cols/2) < MoveParam.green_pixel_range:
                self.topicMsg.toGreenDirection = GREEN_FRONT
              elif self.hit[1] > cols/2:
                self.topicMsg.toGreenDirection = GREEN_RIGHT
              elif self.hit[1] < cols/2:
                self.topicMsg.toGreenDirection = GREEN_LEFT
              else:
                self.topicMsg.toGreenDirection = GREEN_NOTHING
            """
            
          

