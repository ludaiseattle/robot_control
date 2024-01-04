#!/usr/bin/env python

from colorsys import ONE_SIXTH
import rospy
import tf
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random
from sensor_msgs.msg import LaserScan
import traceback

OBSTACLE_AVOID = 1
RH_WALL_FOLLOW = 2
RANDOM_WALK = 3

RAND_DIS = 3

class TurtleNav:
    currentX = None
    currentY = None
    currentTheta = None
    desiredTheta = 0
    linear_speed = 0.1
    angular_speed = 0.2
    forwardFOV = 60
    forwardList = []
    forwardRange = 0.5

    def __init__(self):
        rospy.init_node('TurtleNav', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.on_shutdown(self.onShutdown)
        self.rate = rospy.Rate(300)
        self.distanceTravelled = 0
        self.state = RANDOM_WALK
        self.frontLaser = 0
        self.rightLaser = 0
        self.rightBackLaser = 0
        self.rightFrontLaser = 0
        self.leftLaser = 0
        self.wallEntryThres = 0.6
        self.wallMaxThres = 0.4
        self.wallMinThres = 0.1
        self.rhWallCnt = 0
        self.wallCntThres = 5
        self.wallEntryRBThres = 0.75
        self.wallEntryRFThres = 0.75



    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # why was it pose.theta and so on (error: pose object has no attribute theta)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.currentX == None:
            self.currentX = self.x
        if self.currentY == None:
            self.currentY = self.y
        if self.currentTheta == None:
            self.currentTheta = self.theta

        changex = self.x - self.currentX
        changey = self.y - self.currentY
        changeDis = math.sqrt((changex * changex) + (changey * changey))
        self.distanceTravelled += changeDis

        self.currentX = self.x
        self.currentY = self.y
        self.currentTheta = self.theta

        #print(self.state)

    def scan_callback(self, msg):
        angle = msg.angle_min

        self.frontLaser = msg.ranges[0]
        self.rightLaser = msg.ranges[270]
        self.leftLaser = msg.ranges[90]
        self.rightBackLaser = msg.ranges[240]
        self.rightFrontLaser = msg.ranges[300]

        if len(self.forwardList) > 0:
            del self.forwardList[:]

        for i in range((0 + self.forwardFOV/2), (-1 - self.forwardFOV/2), -1):
            self.forwardList.append(msg.ranges[i%360])
    
    def detectFront(self):
        for x in self.forwardList:
            print(self.forwardList[x])
            if self.forwardList[x] < 0.5:
                return True
        return False
        
    '''    def randWalk(self):
            
            while self.distanceTravelled < RAND_DIS:
                vel = Twist(linear=Vector3(self.linear_speed, 0, 0))
                self.pub.publish(vel)
                self.rate.sleep()
    '''
    def randomRotate(self):
        self.desiredTheta = math.radians(random.randint(0, 360))
        if self.desiredTheta > math.pi:
            self.desiredTheta = math.pi - self.desiredTheta

        while self.currentTheta < self.desiredTheta:
            if self.desiredTheta == math.pi and self.currentTheta < 0:
                # self.desiredTheta = -math.pi
                break
            vel = Twist(angular=Vector3(0, 0, self.angular_speed))
            self.pub.publish(vel)
            self.rate.sleep()
    
        self.distanceTravelled = 0

    def obstacleAvoid(self):
        print("Obstacle avoid")
        self.pub.publish(Twist())
        while self.isObstacle():
            vel = Twist(angular=Vector3(0, 0, self.angular_speed))
            self.pub.publish(vel)
            self.rate.sleep()
        self.pub.publish(Twist(angular=Vector3(0, 0, 0)))
        self.state = RANDOM_WALK

    def rhWallFollow(self):
        print("in wall status!")
        while self.isRhWallFollow():
            if self.isObstacle():
                self.state = OBSTACLE_AVOID
                return

            if self.rightLaser > self.wallMaxThres:
                self.pub.publish(Twist())
                self.pub.publish(Twist(angular=Vector3(0, 0, -self.angular_speed)))
                self.rate.sleep()
            elif self.rightLaser < self.wallMinThres:
                self.pub.publish(Twist())
                self.pub.publish(Twist(angular=Vector3(0, 0, self.angular_speed)))
                self.rate.sleep()
            
            vel = Twist(linear=Vector3(self.linear_speed, 0, 0))
            #vel = Twist(linear = velocity)
            self.pub.publish(vel)
            self.rate.sleep()
        self.state = RANDOM_WALK

    def isObstacle(self):
        for x in self.forwardList:
            #print(x)
            #print(self.forwardList[x])
            if x < 0.5 and not x == 0:
                return True
        return False

    def isRhWallFollow(self):
        '''if self.rightLaser < self.wallDisThres and\
                ++self.rhWallCnt >= self.wallCntThres:'''
        if self.rightLaser < self.wallEntryThres:
            if (self.rightBackLaser < self.wallEntryRBThres) or (self.rightFrontLaser < self.wallEntryRFThres):
                #if self.rightLaser < self.wallEntryThres:
                self.state = RH_WALL_FOLLOW
                #print("in wall status!")
                return True
        else:
            self.rhWallCnt = 0
            return False


    def randomWalk(self):
        print("random walk")
        
        self.distanceTravelled = 0
        while self.distanceTravelled < RAND_DIS:
            if self.isObstacle():
                self.distanceTravelled = 0
                self.state = OBSTACLE_AVOID
                return
            elif self.isRhWallFollow():
                return

            vel = Twist(linear=Vector3(self.linear_speed, 0, 0))
            self.pub.publish(vel)
            self.rate.sleep()
        
        # wasn't working till i added the rotation code back to the randomwalk
        self.desiredTheta = math.radians(random.randint(0, 360))
        if self.desiredTheta > math.pi:
            self.desiredTheta = math.pi - self.desiredTheta


        while self.currentTheta < self.desiredTheta:
            if self.desiredTheta == math.pi and self.currentTheta < 0:
                self.desiredTheta = -math.pi
                break
            vel = Twist(angular=Vector3(0, 0, self.angular_speed))
            self.pub.publish(vel)
            self.rate.sleep()
    
        '''
        if self.state == RANDOM_WALK and self.distanceTravelled < RAND_DIS:
            vel = Twist(linear=Vector3(self.linear_speed, 0, 0))
            self.pub.publish(vel)

        if self.state == RANDOM_WALK and self.distanceTravelled >= RAND_DIS:
            self.randomRotate()
        '''

    def onShutdown(self):
        self.pub.publish(Twist())


    def turtleNav(self):
        while rospy.Time.now().to_sec() == 0:
            pass

        while not rospy.is_shutdown():           
            if self.state == OBSTACLE_AVOID:
                self.obstacleAvoid()
            elif self.state == RH_WALL_FOLLOW:
                self.rhWallFollow()
            elif self.state == RANDOM_WALK:
                self.randomWalk()
            else:
                self.randomWalk()
        rospy.spin()


if __name__ == '__main__':
    try:
        turtleBot = TurtleNav()
        turtleBot.turtleNav()
    except Exception as e:
        rospy.loginfo("Exception entered!")
        rospy.loginfo(e.args)
        rospy.loginfo(traceback.format_exc())
