#!/usr/bin/env python3

import sys
import rospy
import copy
import actionlib
import geometry_msgs.msg
import tf_conversions
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

from math import pi, tau, dist, fabs, cos


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

    def get_quaternion_from_yaw(self, yaw):
        """
        Accept a yaw value and return a quaternion
        """
        return geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))

    def make_base_goal_pose(self, waypoint):
        """
        Accept a waypoint dictionary and return a pose object
        Waypoint dictionary should be of the form:
        {
            "position": {x, y, z},
            "yaw": {a}
        }
        """
        goal_pose = geometry_msgs.msg.Pose()

        goal_pose.position.x = waypoint["position"]["x"]
        goal_pose.position.y = waypoint["position"]["y"]
        goal_pose.orientation = self.get_quaternion_from_yaw(waypoint["yaw"]["a"])

        return goal_pose



