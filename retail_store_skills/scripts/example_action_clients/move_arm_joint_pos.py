#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
import tf
import moveit_commander
from copy import deepcopy

from retail_store_skills.msg import *
from geometry_msgs.msg import PoseStamped
from franka_vacuum_gripper.msg import *
from collision_box_interface import CollisionBoxInterface



class MoveArmJoints(object):
    """
    Class for testing arm joint positions
    """

    def __init__(self, name: str) -> None:


        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity


    def move(self, req: list) -> None:
        rospy.loginfo(f'Moving to {req}')

        self._group.go(req, wait=True)
        rospy.loginfo('Pick action finished moving to start pos')

        return


if __name__ == "__main__":
    arm = MoveArmJoints('move_arm_joint_pos')
    arm.move([0.0, -1.2, 0.0, -2.0, 0.0, 2.0, 0.8])
