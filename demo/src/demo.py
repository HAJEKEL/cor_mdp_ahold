#!/usr/bin/env python3

import sys
import rospy
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from move_group_python_interface import MoveGroupPythonInterface

from math import pi, tau, dist, fabs, cos

# Moving the robot base to a position in front of the shelf
class MoveBase(object): 
    def __init__(self):
        #Launch client and wait for server
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()

    def run(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = goal_pose

        rospy.loginfo("Sending move base goal ...")
        self.client.send_goal(goal)
        success = self.client.wait_for_result()


        if success:
            rospy.loginfo("Reached goal!")
        else:
            rospy.loginfo("Failed to reach goal for some reason")



def get_quaternion_from_yaw(yaw):
    return geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))

def make_base_goal_pose(x: float, y: float, yaw: float):
    goal_pose = geometry_msgs.msg.Pose()

    goal_pose.position.x = x
    goal_pose.position.y = y
    goal_pose.orientation = get_quaternion_from_yaw(yaw)

    return goal_pose









if __name__ == '__main__':
    rospy.init_node('demo')
    rospy.loginfo("Starting demo node")

    # Create a MoveBase object
    move_base = MoveBase()

    # Move the robot to the first position
    rospy.loginfo("Moving to base station")
    move_base.run(make_base_goal_pose(4.0, 4.0, 0.0))

    # Move the robot to the second position
    rospy.loginfo("Moving to shelf")
    move_base.run(make_base_goal_pose(-2.5, -0.5, 3.14))



    rospy.loginfo("Demo finished")



