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


class PickActionServer(object):
    def __init__(self, name: str, pick_distance: float = 0.2) -> None:
        # Read config
        self.simulation = True 
        #rospy.get_param("simulation")

        # Initialize action server
        self._rate = rospy.Rate(20)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PickAction, execute_cb=self.as_cb, auto_start=False)

        self.pick_distance = pick_distance

        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity
        self._tl = tf.TransformListener()

        # Collision box interface
        self._collision_box_interface = CollisionBoxInterface()

        # Initialize vacuum gripper action client and related variable
        self._vacuum_client = actionlib.SimpleActionClient("/franka_vacuum_gripper/vacuum", VacuumAction)
        rospy.loginfo(f'Waiting for vacuum server')
        self._vacuum_client.wait_for_server()
        rospy.loginfo(f'Vacuum server found')

        self.vacuum_state_subscriber = rospy.Subscriber(
            "/franka_vacuum_gripper/vacuum_state", VacuumState, self.vacuum_state_cb
        )

        self._as.start()

    def vacuum_state_cb(self, msg: VacuumState):
        self._vacuum_state = msg


    def as_cb(self, req):
        rospy.loginfo(f'Pick action called for Tag {req.goal_id}')

        if self._vacuum_state.part_present:
            rospy.loginfo(f'Part already present, pick action aborted!')
            self._as.set_aborted()
            return
        
        # Step 0: Add collision boxes
        self._collision_box_interface.add_basket_box()
        #self._collision_box_interface.add_shelf_collision_boxes(req.goal_id)

        # Step 1: Joint goal start position
        rospy.loginfo('Pick action moving to start pos')
        start_pos = [0.0, -0.8, 0.0, -2.5, 0.0, 3.1, 0.8]
        self._group.go(start_pos, wait=True)
        rospy.loginfo('Pick action finished moving to start pos')

        # Step 2: Position ee in front of requested pick tag
        pre_pose = PoseStamped()
        pre_pose.pose.position.z = self.pick_distance
        pre_pose.pose.orientation.x = 1
        pre_pose.pose.orientation.w = 0
        pre_pose.header.frame_id = "tag_{}".format(req.goal_id)

        self._group.set_pose_target(pre_pose)
        succeeded = self._group.go(wait=True)
        rospy.loginfo(f"Pick action pre-pose {'succeeded' if succeeded else 'failed'}")
        if not succeeded or self._as.is_preempt_requested():
            self._as.set_aborted()
            return

        # Step 3: Cartesian forward
        goal = pre_pose
        goal.pose.position.z -= (self.pick_distance + 0.02)
        goal_map_frame = self._tl.transformPose("map", goal) # Because compute_cartesian_path assumes poses in planning_frame(=map)
        (plan, _) = self._group.compute_cartesian_path([goal_map_frame.pose], 0.01, 0.0)
        succeeded = self._group.execute(plan, wait=True)
        rospy.loginfo(f"Pick action forward {'succeeded' if succeeded else 'failed'}")
        if not succeeded or self._as.is_preempt_requested():
            self._as.set_aborted()
            return

        # step 3: start vacuum pump
        goal = VacuumGoal(vacuum=10)
        res = self._vacuum_client.send_goal(goal)

        if not succeeded or self._as.is_preempt_requested():
            self._as.set_aborted()
            return

        # Step 4: Add product collision box
        #self._collision_box_interface.add_product_box()
        #self._collision_box_interface.attach_box()


        # Step 5: Cartesian up and backwards
        goal = self._group.get_current_pose().pose
        goal.position.z += 0.05
        (plan, _) = self._group.compute_cartesian_path([goal], 0.01, 0.0)
        succeeded = self._group.execute(plan, wait=True)
        rospy.loginfo(f"Pick action up {'succeeded' if succeeded else 'failed'}")
        if not succeeded or self._as.is_preempt_requested():
            self._as.set_aborted()
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose = self._group.get_current_pose().pose
        goal = self._tl.transformPose(f"tag_{req.goal_id}", goal) 
        goal.pose.position.z += 0.15
        goal = self._tl.transformPose("map", goal) 
        (plan, _) = self._group.compute_cartesian_path([goal.pose], 0.02, 0.0)
        succeeded = self._group.execute(plan, wait=True)
        rospy.loginfo(f"Pick action backwards {'succeeded' if succeeded else 'failed'}")
        if not succeeded or self._as.is_preempt_requested():
            self._as.set_aborted()
            return

        # Step 6: go back to holding pose
        holding_pos = [-0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8] 
        succeeded = self._group.go(holding_pos, wait=True)
        rospy.loginfo(f"Pick action holding pose {'succeeded' if succeeded else 'failed'}")

        if succeeded and not self._as.is_preempt_requested():
            self._as.set_succeeded()
        else:
            self._as.set_aborted()

        self._collision_box_interface.remove_box(box_name="basket_box")
        #self._collision_box_interface.remove_shelf_collision_boxes(box_name="product_box")

        return


if __name__ == "__main__":
    rospy.init_node("pick_server")
    PickActionServer(rospy.get_name())
    rospy.spin()
