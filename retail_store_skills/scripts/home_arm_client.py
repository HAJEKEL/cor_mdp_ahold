#!/usr/bin/env python3

import rospy
import actionlib
from retail_store_skills.msg import HomeArmAction, HomeArmGoal



class HomeArmClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/home_arm_server", HomeArmAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

    def run(self):
        goal = HomeArmGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        # if success, print picking finished
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Homing finished")
        else:
            rospy.loginfo("Homing failed")


if __name__ == "__main__":
    rospy.init_node("test")
    client = HomeArmClient()
    client.run()
