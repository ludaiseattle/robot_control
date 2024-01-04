import rospy
import actionlib
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from IAction import IAction
from common import MoveParam
from Logger import logger
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from Global import *

class MoveToGoal(IAction):
    def __init__(self, topicMsg):
        super(MoveToGoal, self).__init__("MoveToGoal")
        self.topicMsg = topicMsg
        self.pub = self.topicMsg.pub
        self.goal = Pose()
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.topicMsg.move_base = self.ac

        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")

    def execute(self, pose):
        

        self.goal = pose
        dest = MoveBaseGoal()

        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        dest.target_pose.pose = self.goal

        self.ac.send_goal(dest)
        self.ac.wait_for_result(rospy.Duration(300))
        #print("status = ", self.ac.get_state(), "dest=", dest)
        if(self.ac.get_state() ==  GoalStatus.SUCCEEDED):
              return True
        else:
              return False

    def getPoint(self, x, y):
        pose = Pose()
        pose.position = Point(x, y, 0)
        orientation = tf.transformations.quaternion_from_euler(0, 0, self.topicMsg.currentTheta)
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        #pose.orientation = orientation
        return pose

    def cancelGoal():
        self.ac.cancelGoal()

    def cancelAllGoal():
        self.ac.cancelAllGoals()

