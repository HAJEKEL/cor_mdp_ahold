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
        # if success, print picking finished
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Picking finished")
        else:
            rospy.loginfo("Picking failed")


if __name__ == "__main__":
    rospy.init_node("test")
    id = int(sys.argv[1])
    rospy.loginfo("Pick up apriltag id: %d", id)
    client = PickClient()
    client.run(id)
