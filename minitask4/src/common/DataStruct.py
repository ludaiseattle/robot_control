from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from Global import *

class MoveParam():
    desiredTheta = 0
    wallEntryThres = 1.0
    wallMaxThres = 0.7
    wallMinThres = 0.4
    wallCntThres = 5
    wallEntryRBThres = 1.25
    wallEntryRFThres = 1.25
    RAND_DIS = 2
    linear_speed = 0.15
    angular_speed = 0.1
    runToGreen_switch = 1
    obstacle_dis = 0.5
    p_divider = 1000

class OccupiedGridParams():
    resolution = 0.1
    width = 200 # the index of cell in width
    height = 200 # the index of cell in height
    origin = Pose(Point(-10,-10,0),Quaternion(0,0,0,0.0)) 
    occupied_thresh = 3
    free_thresh = 1

class TopicMsg():
    def __init__(self):
        self.distanceTravelled = 0
        self.currentTheta = None
        self.pub = None
        self.rate = None
        self.frontLaser = 0
        self.rightLaser = 0
        self.rightBackLaser = 0
        self.rightFrontLaser = 0
        self.leftLaser = 0
        self.lastState = None
        self.currentX = None
        self.currentY = None
        self.forwardFOV = 56
        self.forwardList = []
        self.laser_range_min = None
        self.laser_range_max = None
        self.scan_msg = None
        self.odom_msg = None


