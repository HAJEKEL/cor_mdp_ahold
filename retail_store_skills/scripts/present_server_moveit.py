#!/usr/bin/env python3 

#ROS packages
import rospy
import actionlib
import tf #part of robot_state_publisher support for 3D transformations
import moveit_commander #Python api for moveit

#Scientific Computing and Numerical Operations
import numpy as np #import numpy with alias np

#message types 
from retail_store_skills.msg import * #PresentAction and DropOffAction from retail_store_skills
from retail_store_skills.msg import PresentAction

from franka_vacuum_gripper.msg import * #VacuumState message type is used
from geometry_msgs.msg import PoseStamped #PoseStamped message type is used


class PresentActionServer(object):
    def __init__(self, name: str) -> None:
        # Read config
        self.simulation = True  # rospy.get_param("simulation")

        # Initialize action server
        self._rate = rospy.Rate(20)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, PresentAction, execute_cb=self.as_cb, auto_start=False
        )

        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6)  # Increase group velocity
        self._tl = tf.TransformListener()

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
        rospy.loginfo(f"Present action called")

        if not self._vacuum_state.part_present:
           rospy.loginfo(f"Part not present, present action aborted!")
           self._as.set_aborted()
           return

        
        # Step 1: Joint goal start position
        rospy.loginfo("Present action moving to start pos")
        start_pos = [-0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8]
        succeeded = self._group.go(start_pos, wait=True)
        rospy.loginfo("Present action finished moving to start pos")

        if not succeeded:
            self._as.set_aborted()
            return

        # Step 2: move joints to present position
        present_pos = [-1.57, -0.8, 0.0, -2.3, 0.0, 2.7, 0.8]
        succeeded = self._group.go(present_pos, wait=True)

        if not succeeded:
            self._as.set_aborted()
            return

        # Step 3: wait until part not present in gripper
        while self._vacuum_state.part_present:
            rospy.loginfo("Part present, waiting...")
            self._rate.sleep()
       
       # Step 4: if part not present, initialise dropoff
        rospy.loginfo(f"Dropping it off")
        goal = DropOffGoal(timeout=10)
        self._dropoff_client.send_goal(goal)

        rospy.loginfo(f"Part has been picked by customer")

        return


if __name__ == "__main__":
    rospy.init_node("present_server")
    PresentActionServer(rospy.get_name())
    rospy.spin()
