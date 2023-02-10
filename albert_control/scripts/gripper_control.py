#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib

import numpy as np


if __name__ == "__main__":
    rospy.init_node('gripper_control')

    # gripper_control_pub = rospy.Publisher('panda_simulator/panda_gripper_controller/follow_joint_trajectory', FollowJointTrajectory, queue_size=10)

    client = actionlib.SimpleActionClient("panda_simulator/panda_gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo("server connected")

    joint_names = ['panda_finger_joint1', 'panda_finger_joint2']

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0, 0.0]
    trajectory.points[0].velocities = [0.0 for i in joint_names]
    trajectory.points[0].effort = [0.0 for i in joint_names]
    trajectory.points[0].accelerations = [0.0 for i in joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(2.0)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    client.send_goal(goal)
    rospy.loginfo(client.wait_for_result(rospy.Duration(3.0)))