from geometry_msgs.msg import Twist
from Global import *

class MoveParam():
    desiredTheta = 0
    wallEntryThres = 0.6
    wallMaxThres = 0.4
    wallMinThres = 0.2
    wallCntThres = 5
    wallEntryRBThres = 0.75
    wallEntryRFThres = 0.75
    RAND_DIS = 3
    linear_speed = 0.15
    angular_speed = 0.1
    runToGreen_switch = 1
    obstacle_dis = 0.5
    p_divider = 1000
    green_pixel_range = 50
    found_green = 0

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
        self.forwardFOV = 60
        self.forwardList = []
        self.greenVel = Twist()
        self.toGreenDirection = GREEN_NOTHING

