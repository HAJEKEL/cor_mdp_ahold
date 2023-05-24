#!/usr/bin/env python3

import rospy
import actionlib
from retail_store_skills.msg import PlaceAction
from retail_store_skills.msg import PlaceGoal
from geometry_msgs.msg import PoseStamped


class PlaceBasketClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/place_server", PlaceAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

    def run(self):
        goal = PlaceGoal()
        goal.goal = PoseStamped()

        # Goal location in panda frame
        goal.goal.header.frame_id = "panda_link0"
        goal.goal.pose.position.x = -0.4
        goal.goal.pose.position.y = 0.0
        goal.goal.pose.position.z = 0.5
        goal.goal.pose.orientation.x = -0.105
        goal.goal.pose.orientation.y = 0.994
        goal.goal.pose.orientation.z = 0
        goal.goal.pose.orientation.w = -0.035




        rospy.loginfo("Sending placing goal...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo("Placing finished")


if __name__ == "__main__":
    rospy.init_node("test")
    client = PlaceBasketClient()
    client.run()

