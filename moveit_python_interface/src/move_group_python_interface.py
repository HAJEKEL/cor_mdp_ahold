#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# using Moveit's move_group interface to move the robot arm
class MoveGroupPythonInterface(object):
    """Interface to move the arm using python"""

    def __init__(self):
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface")

        self.robot = moveit_commander.RobotCommander()
        #scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # getting other info for debugging
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


    def print_info(self):
        ## Getting Basic Information
        # We can get the name of the reference frame for this robot:
        print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        print("============ End effector link: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def pick(self):
        ## Creating and executing a picking action using a grasp message

        grasps = []
        g = moveit_msgs.msg.Grasp()

        # Setting grasp pose
        g.grasp_pose.header.frame_id = "panda_vacuum"
        g.grasp_pose.pose.position.x = 0.3
        g.grasp_pose.pose.position.y = 0.0
        g.grasp_pose.pose.position.z = 0.5
        g.grasp_pose.pose.orientation.w = 1.0

        # Setting pre-grasp approach
        g.pre_grasp_approach.direction.header.frame_id = "panda_vacuum"
        g.pre_grasp_approach.direction.vector.z = 1.0
        g.pre_grasp_approach.min_distance = 0.095
        g.pre_grasp_approach.desired_distance = 0.115

        # Setting post-grasp retreat
        g.post_grasp_retreat.direction.header.frame_id = "panda_vacuum"
        g.post_grasp_retreat.direction.vector.z = 1.0
        g.post_grasp_retreat.min_distance = 0.1
        g.post_grasp_retreat.desired_distance = 0.25

        # Setting post-place retreat
        g.post_place_retreat.direction.header.frame_id = "panda_vacuum"
        g.post_place_retreat.direction.vector.z = 1.0
        g.post_place_retreat.min_distance = 0.1
        g.post_place_retreat.desired_distance = 0.25

        # Calling the planner to compute the plan and execute it.
        grasps.append(g)
        self.move_group.pick("object", grasps)




if __name__ == "__main__":
    try:
        move_group_python_interface = MoveGroupPythonInterface()
        move_group_python_interface.print_info()
        move_group_python_interface.pick()
    except rospy.ROSInterruptException:
        pass





