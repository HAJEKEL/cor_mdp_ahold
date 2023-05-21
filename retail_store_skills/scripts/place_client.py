#!/usr/bin/env python3

import rospy
import actionlib
from retail_store_skills.msg import PlaceAction
from retail_store_skills.msg import PlaceGoal
from geometry_msgs.msg import Pose


class PlaceClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/place_server", PlaceAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

    def run(self):
        goal = PlaceGoal()
        goal.goal = Pose()

        # Goal location in map frame
        goal.goal.position.x = 0
        goal.goal.position.y = 0
        goal.goal.position.z = 1.0

        # Goal orientation in map frame
        # q = [ 0, 0.7071068, 0, 0.7071068 ] # aligned with map x-axis
        # q = [ -0.7071068, 0, 0, 0.7071068 ] # aligned with map y-axis
        q = [1, 0, 0, 0]  # Top down
        goal.goal.orientation.x = q[0]
        goal.goal.orientation.y = q[1]
        goal.goal.orientation.z = q[2]
        goal.goal.orientation.w = q[3]

        rospy.loginfo("Sending placing goal...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo("Placing finished")


if __name__ == "__main__":
    rospy.init_node("test")
    client = PlaceClient()
    client.run()
