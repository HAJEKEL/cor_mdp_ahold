#!/usr/bin/env python3

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf_conversions
from geometry_msgs.msg import Pose, Quaternion

def get_quaternion_from_yaw(yaw):
        """
        Accept a yaw value and return a quaternion
        """
        return Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))

class MoveBase(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo('Waiting for server...')
        self.client.wait_for_server()

    def run(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = goal_pose

        rospy.loginfo('Sending move base goal...')
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo('Navigation finished')


if __name__ == '__main__':
    rospy.init_node("test")

    move_base = MoveBase()
    goal1 = Pose()
    goal2 = Pose()

    goal1.position.x = 0.0
    goal1.position.y = 0.0
    goal1.position.z = 0.0
    goal1.orientation = get_quaternion_from_yaw(1.57)

    goal2.position.x = 1.1
    goal2.position.y = -0.0
    goal2.position.z = 0.0
    goal2.orientation = get_quaternion_from_yaw(0.0)

    move_base.run(goal1)
    move_base.run(goal2)


