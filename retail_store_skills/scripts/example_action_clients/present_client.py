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
        goal.goal_id = 0
        
        rospy.loginfo("Present goal client running...")
        

        rospy.loginfo("Sending presenting goal...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # if success, print presenting finished
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Presenting finished")

        else:
            rospy.loginfo("Presenting failed")

        return


if __name__ == "__main__":
    rospy.init_node("test")
    client = PresentClient()
    client.run()
