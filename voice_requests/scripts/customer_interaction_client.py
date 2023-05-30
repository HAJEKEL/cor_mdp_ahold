#!/usr/bin/env python3

import rospy
import actionlib
import sys
from voice_requests.msg import CustomerInteractionAction, CustomerInteractionGoal



class CustomerInteractionClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/customer_interaction_server", CustomerInteractionAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

    def run(self):
        # Send goal
        goal = CustomerInteractionGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        # if success, print picking finished
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Customer interaction finished")
        else:
            rospy.loginfo("Customer interaction failed")


if __name__ == "__main__":
    rospy.init_node("test")
    client = CustomerInteractionClient()
    client.run()
