#!/usr/bin/env python3

# ROS packages
import rospy
import actionlib
import tf  # part of robot_state_publisher support for 3D transformations
import moveit_commander  # Python API for MoveIt

# Message types
from retail_store_skills.msg import *
from franka_vacuum_gripper.msg import VacuumState, DropOffGoal, DropOffAction
import tf2_ros
import tf2_geometry_msgs
import math

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

        # MoveIt interfaces
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

        # if not self._vacuum_state.part_present:
        #     rospy.loginfo(f"Part not present, present action aborted!")
        #     self._as.set_aborted()
        #     return

        # Step 1: Get the current joint positions
        joint_positions = self._group.get_current_joint_values()

        # Step 2: Get the cluster center position from TFMessage
        cluster_center = self.get_cluster_center_from_tf()

        if cluster_center is None:
            rospy.loginfo(f"Cluster center not found, present action aborted!")
            self._as.set_aborted()
            return

        # Step 3: Calculate the rotation angle based on cluster center position
        target_x = cluster_center[0]
        target_y = cluster_center[1]
        current_x = joint_positions[0]  # Assuming the current rotation angle is stored in the first joint position

        rotation_angle = math.atan2(target_y - current_y, target_x - current_x)

        # Step 4: Rotate the first joint by the calculated angle
        joint_positions[0] += rotation_angle

        # Step 5: Set the target joint positions and move the arm
        self._group.set_joint_value_target(joint_positions)
        succeeded = self._group.go(wait=True)

        if not succeeded:
            self._as.set_aborted()
            return

        # Step 6: wait until part not present in gripper
        while self._vacuum_state.part_present:
            rospy.loginfo("Part present, waiting...")
            self._rate.sleep()

        # Step 7: if part not present, initialize dropoff
        rospy.loginfo(f"Dropping it off")
        goal = DropOffGoal(timeout=10)
        self._dropoff_client.send_goal(goal)

        # Step 8: return to home position
        succeeded = self._group.go(start_pos, wait=True)

        if not succeeded:
            self._as.set_aborted()
            return

        rospy.loginfo(f"Part has been picked by customer")

        self._as.set_succeeded()

    def get_cluster_center_from_tf(self):
        try:
            # Retrieve the latest transform from "base_link" to "frame_cluster_1"
            trans = self._tl.lookup_transform("base_link", "frame_cluster_1", rospy.Time())
            cluster_center = [trans.transform.translation.x, trans.transform.translation.y]
            return cluster_center
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Error retrieving cluster center from TF")
            return None


if __name__ == "__main__":
    rospy.init_node("present_server")
    PresentActionServer(rospy.get_name())
    rospy.spin()

