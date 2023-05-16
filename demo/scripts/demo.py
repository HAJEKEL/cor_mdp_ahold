#!/usr/bin/env python3

import sys
import rospy
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from closest_link_attacher.srv import Attach, AttachRequest, AttachResponse, Detach, DetachRequest

from franka_vacuum_gripper.msg import *
from franka_vacuum_gripper.msg import *

from plan_arm_ik import PlanArmIK
from move_base import MoveBase


from math import pi, tau, dist, fabs, cos

def pick_product(arm_planner, pick_loc, dropoff_loc):
    # Home arm
    arm_planner.home()

    # Move arm to shelf
    rospy.loginfo("Planning arm movement to product shelf 1")
    arm_shelf_goal = arm_planner.make_arm_goal_pose(ee_pose=pick_loc, vacuum_orientation="forward")
    rospy.loginfo(arm_shelf_goal)
    arm_planner.run(arm_shelf_goal)

    # Activate vacuum
    rospy.loginfo("Activating vacuum")
    arm_planner.activate_vacuum()

    # Move arm to basket
    rospy.loginfo("Planning arm movement to basket dropoff")
    arm_basket_goal = arm_planner.make_arm_goal_pose(dropoff_loc)
    rospy.loginfo(arm_basket_goal)
    arm_planner.run(arm_basket_goal)

    # Deactivate vacuum
    rospy.loginfo("Deactivating vacuum")
    arm_planner.deactivate_vacuum()

def test_arm_location(arm_planner, pick_loc):
    arm_planner.home()
    arm_shelf_goal = arm_planner.make_arm_goal_pose(ee_pose=pick_loc, vacuum_orientation="forward")
    rospy.loginfo(arm_shelf_goal)
    arm_planner.run(arm_shelf_goal)


if __name__ == '__main__':
    rospy.init_node('demo')
    rospy.loginfo("Starting demo node")
    
    # Initialize arm planner
    arm_planner = PlanArmIK(vis_pub=False)

    # Initialize a base movement client
    move_base = MoveBase()


    #### Get product with arm ####
    basket_dropoff = rospy.get_param("demo_params/ee_poses/basket_dropoff")
    basket_false_dropoff = rospy.get_param("demo_params/ee_poses/basket_false_dropoff")
    product_shelf_1 = rospy.get_param("demo_params/ee_poses/product_shelf_1")
    product_shelf_2 = rospy.get_param("demo_params/ee_poses/product_shelf_2")

    # input("============ Press `Enter` to execute test the arm picking location")
    # test_arm_location(arm_planner, product_shelf_2)

    input("============ Press `Enter` to execute the first pick")
    pick_product(arm_planner, product_shelf_1, basket_dropoff)

    input("============ Press `Enter` to execute the second pick")
    pick_product(arm_planner, product_shelf_2, basket_false_dropoff)


    input("============ Press `Enter` to move to base station")
    # get base station waypoint from param server
    base_station = rospy.get_param("demo_params/waypoints/base_station")
    shelf_1 = rospy.get_param("demo_params/waypoints/shelf_1")

    # Move the robot to the base station
    rospy.loginfo("Moving to base station")
    move_base.run(move_base.make_base_goal_pose(base_station))

    # Move the robot to the second position
    rospy.loginfo("Moving to shelf")
    move_base.run(move_base.make_base_goal_pose(shelf_1))



    rospy.loginfo("Demo finished")



