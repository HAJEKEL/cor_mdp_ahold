#!/usr/bin/env python3

import rospy
import actionlib
import sys
from retail_store_skills.msg import PickAction
from retail_store_skills.msg import PickGoal



class PickClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/pick_server", PickAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

    def run(self, apriltag_id):
        goal = PickGoal()
        goal.goal_id = apriltag_id

        rospy.loginfo("Sending picking goal...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo("Picking finished")


if __name__ == "__main__":
    rospy.init_node("test")
    client = PickClient()
    client.run(18)
