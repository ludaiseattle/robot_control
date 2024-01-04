import rospy
import actionlib
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
        
        

    def execute(self, pose):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")

        self.goal = pose
        dest = MoveBaseGoal()

        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        dest.target_pose.pose = self.goal

        ac.send_goal(dest)
        ac.wait_for_result(rospy.Duration(300))
        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              return True
        else:
              return False
