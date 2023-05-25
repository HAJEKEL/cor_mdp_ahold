#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
import tf
import moveit_commander

from retail_store_skills.msg import *
from franka_vacuum_gripper.msg import *
from geometry_msgs.msg import PoseStamped
from collision_box_interface import CollisionBoxInterface


class PlaceActionServer(object):
    def __init__(self, name: str) -> None:
        # Read config
        self.simulation = True  # rospy.get_param("simulation")

        # Initialize action server
        self._rate = rospy.Rate(20)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, PlaceAction, execute_cb=self.as_cb, auto_start=False
        )

        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6)  # Increase group velocity
        self._tl = tf.TransformListener()

        # Collision box interface
        self._collision_box_interface = CollisionBoxInterface()

        # Initialize vacuum gripper action client and related variable
        self._dropoff_client = actionlib.SimpleActionClient(
            "/franka_vacuum_gripper/dropoff", DropOffAction
        )
        rospy.loginfo(f"Waiting for dropoff server")
        self._dropoff_client.wait_for_server()
        rospy.loginfo(f"dropoff server found")

        self.vacuum_state_subscriber = rospy.Subscriber(
            "/franka_vacuum_gripper/vacuum_state", VacuumState, self.vacuum_state_cb
        )

        self._as.start()

    def vacuum_state_cb(self, msg: VacuumState):
        self._vacuum_state = msg

    def as_cb(self, req):
        rospy.loginfo(f"Place action called")

        if not self._vacuum_state.part_present:
           rospy.loginfo(f"Part not present, place action aborted!")
           self._as.set_aborted()
           return

        self._collision_box_interface.add_basket_box()
        # self._collision_box_interface.add_product_box()
        # self._collision_box_interface.attach_box(box_name="product_box")


        
        # Step 1: Joint goal start position
        rospy.loginfo("Place action moving to start pos")
        start_pos = [-0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8]
        succeeded = self._group.go(start_pos, wait=True)
        rospy.loginfo("Place action finished moving to start pos")

        if not succeeded:
            # self._collision_box_interface.detach_box(box_name="product_box")
            # self._collision_box_interface.remove_box(box_name="product_box")
            self._as.set_aborted()
            return

        # Step 2: Cartesian goal to final drop position in map frame
        rospy.loginfo("Place action moving to final drop pos")
        pose = PoseStamped()
        pose.header.frame_id = req.goal.header.frame_id
        pose.pose = req.goal.pose

        # check for reachability, and give warning
        test_pose = self._tl.transformPose("panda_link0", pose)
        test_dist = np.linalg.norm(np.array([test_pose.pose.position.x, test_pose.pose.position.y, test_pose.pose.position.z]))
        if test_dist > 0.8:
            rospy.logwarn(f"The specified place goal is {test_dist:.2f} meters away from the base of the arm, and thus likely out of reach!")

        # sometimes the planning fails, so we try a few times
        for i in range(3):
            self._group.set_pose_target(pose)
            succeeded = self._group.go(wait=True)
            if succeeded:
                break
            else:
                rospy.logwarn(f"Planning failed, trying again...")

        if not succeeded:
            self._collision_box_interface.detach_box(box_name="product_box")
            self._collision_box_interface.remove_box(box_name="product_box")
            self._as.set_aborted()
            return

        # Step 3: Dropoff
        rospy.loginfo(f"Dropping it off")
        goal = DropOffGoal(timeout=10)
        self._dropoff_client.send_goal(goal)

        if succeeded:
            self._as.set_succeeded()
        else:
            self._collision_box_interface.detach_box(box_name="product_box")
            self._collision_box_interface.remove_box(box_name="product_box")
            self._as.set_aborted()

        # Step 4: Remove product collision boxes
        self._collision_box_interface.detach_box(box_name="product_box")
        

        return


if __name__ == "__main__":
    rospy.init_node("place_server")
    PlaceActionServer(rospy.get_name())
    rospy.spin()
