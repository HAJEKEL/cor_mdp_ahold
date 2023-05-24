#!/usr/bin/env python3

import rospy
import actionlib
from retail_store_skills.msg import ScanShelfAction, ScanShelfGoal


class ScanClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/scan_shelf_server", ScanShelfAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

    def run(self):
        goal = ScanShelfGoal()
        goal.shelf_direction = 0.0

        rospy.loginfo("Sending placing goal...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo("Scanning Finished")


if __name__ == "__main__":
    rospy.init_node("test")
    client = ScanClient()
    client.run()
