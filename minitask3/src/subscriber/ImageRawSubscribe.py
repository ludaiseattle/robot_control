import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
from cv2 import blur
import numpy 
import math
from Logger import logger
from geometry_msgs.msg import Twist
from common import MoveParam
from Global import *

class ImageRawSubscribe():
    def __init__(self, rospy, topicMsg):
        self.printimg = True
        self.rospy = rospy
        self.topicMsg = topicMsg
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("original", 1)
        self.image_sub = self.rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        self.result = None
        self.stopCheck = None
        self.hit = None
        self.pixelRange = 50

    def image_callback(self, msg):
          self.result = None
          image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
          (h, w) = image.shape[:2]
          image_resized = cv2.resize(image, (w/4,h/4))
          self.result = image_resized.copy()
          hsv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

          lower = numpy.array([60, 70, 70])
          upper = numpy.array([110, 255, 255])

          mask = cv2.inRange(hsv_image, lower, upper)
          self.result = cv2.bitwise_and(self.result, self.result, mask=mask)

          cv2.imshow('mask', mask)
          cv2.imshow('result', self.result)
          # cv2.imshow("original", image_resized)
          # cv2.imshow("hsv", hsv_image)
          
          h, w = self.result.shape[:2]
          rows,cols,_ = self.result.shape
          if self.hit != None:
            check = self.result[self.hit[1], self.hit[0]]
            if check[2] != 0:
              pass
            else:
              self.hit = None

          if self.hit == None:
            for y in range(rows):
              k = self.result[y,w/2]
              if k[2] != 0:
                        logger.debug("location of green: i-" + str(y) + " j-" + str(w/2))
                        self.hit = (w/2,y)
                        break

          if self.hit == None:
              logger.debug("get new green hit!")
              for i in range(rows):
                for j in range(cols):
                    k = self.result[i,j]
                    if k[2] != 0:
                        logger.debug("location of green: i-" + str(i) + " j-" + str(j))
                        self.hit = (j,i)
                        break

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

          if self.hit == None:
                self.topicMsg.greenVel = Twist()
                self.topicMsg.toGreenDirection = "nothing"
                logger.debug("direction: " + self.topicMsg.toGreenDirection)
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
            
          cv2.waitKey(3)

