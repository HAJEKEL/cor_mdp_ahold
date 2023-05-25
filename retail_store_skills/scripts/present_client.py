#!/usr/bin/env python3

import rospy
import actionlib
from retail_store_skills.msg import PresentAction
from retail_store_skills.msg import PresentGoal
from geometry_msgs.msg import Pose, PoseStamped


class PresentClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/present_server", PresentAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

    def run(self):
        goal = PresentGoal()
        goal.goal = PoseStamped()

        # Goal location in map frame
        goal.goal.pose.position.x = 0
        goal.goal.pose.position.y = 0
        goal.goal.pose.position.z = 1.0

        # Goal orientation in map frame
        # q = [ 0, 0.7071068, 0, 0.7071068 ] # aligned with map x-axis
        # q = [ -0.7071068, 0, 0, 0.7071068 ] # aligned with map y-axis
        q = [1, 0, 0, 0]  # Top down
        goal.goal.pose.orientation.x = q[0]
        goal.goal.pose.orientation.y = q[1]
        goal.goal.pose.orientation.z = q[2]
        goal.goal.pose.orientation.w = q[3]

        # Goal frame
        goal.goal.header.frame_id = "map"

        rospy.loginfo("Sending presenting goal...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo("Presenting finished")


if __name__ == "__main__":
    rospy.init_node("test")
    client = PresentClient()
    client.run()
