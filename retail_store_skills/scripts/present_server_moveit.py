#!/usr/bin/env python3

import rospy
import actionlib
import tf
import moveit_commander
from retail_store_skills.msg import *
from franka_vacuum_gripper.msg import VacuumState, DropOffGoal, DropOffAction
import tf2_ros
import tf2_geometry_msgs
import math
from std_msgs.msg import Bool
import geometry_msgs.msg


class PresentActionServer(object):
    def __init__(self, name: str)->None:
        self.simulation = True  # rospy.get_param("simulation")

        # Set the rate at which the node operates
        self._rate = rospy.Rate(20)

        # Initialize the action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, PresentAction, execute_cb=self.as_cb, auto_start=False
        )

        # Subscribe to the customer detection topic
        rospy.Subscriber('/customer_detected', Bool, self.customer_detected_callback)

        # Initialize MoveIt components
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6)

        # Initialize TF listener
        self._tl = tf.TransformListener()

        # Initialize the drop-off action client
        self._dropoff_client = actionlib.SimpleActionClient(
            "/franka_vacuum_gripper/dropoff", DropOffAction
        )
        rospy.loginfo(f"Waiting for dropoff server")
        self._dropoff_client.wait_for_server()
        rospy.loginfo(f"dropoff server found")

        # Subscribe to the vacuum state topic
        self.vacuum_state_subscriber = rospy.Subscriber(
            "/franka_vacuum_gripper/vacuum_state", VacuumState, self.vacuum_state_cb
        )

        # Start the action server
        self._as.start()

    def customer_detected_callback(self, msg):
        # Callback function for customer detection updates
        self.customer_detected = msg.data

    def vacuum_state_cb(self, msg: VacuumState):
        # Callback function for vacuum state updates
        self._vacuum_state = msg

    def as_cb(self, req):
        rospy.loginfo(f"Present action called")

        # Check if the part is present before starting the action
        if not self._vacuum_state.part_present:
            rospy.loginfo(f"Part not present, present action aborted!")
            self._as.set_aborted()
            return

        rospy.loginfo("Present action moving to start position")

        # Move the arm to the start position
        start_pos = [-0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8]
        succeeded = self._group.go(start_pos, wait=True)

        rospy.loginfo("Present action finished moving to start position")

        # Check if the arm successfully moved to the start position
        if not succeeded:
            rospy.loginfo(f"Not able to move to start position, present action aborted!")
            self._as.set_aborted()
            return

        # Get the cluster center from the first cluster
        cluster_center_1 = self.get_cluster_center_from_tf("frame_cluster_1")
        rospy.loginfo(f"Cluster center from the first cluster obtained")

        if cluster_center_1 is not None:
            # Point the arm to the cluster center of the first cluster
            self.point_arm_to_cluster(cluster_center_1)

            # Check if a customer is detected in the first cluster
            customer_detected_1 = self.customer_detection()

            if customer_detected_1:
                rospy.loginfo("Customer detected in cluster 1")
                cluster_center = cluster_center_1
            else:
                rospy.loginfo("No customer detected in cluster 1, checking cluster 2...")
        else:
            rospy.loginfo("Cluster 1 center not found, checking cluster 2...")

            # Get the cluster center from the second cluster
            cluster_center_2 = self.get_cluster_center_from_tf("frame_cluster_2")
            rospy.loginfo(f"Cluster center from the second cluster obtained")

            if cluster_center_2 is not None:
                # Point the arm to the cluster center of the second cluster
                self.point_arm_to_cluster(cluster_center_2)

                # Check if a customer is detected in the second cluster
                customer_detected_2 = self.customer_detection()

                if customer_detected_2:
                    rospy.loginfo("Customer detected in cluster 2")
                    cluster_center = cluster_center_2
                else:
                    rospy.loginfo("No customer detected in cluster 2, present action aborted!")
                    self._as.set_aborted()
                    return
            else:
                rospy.loginfo("Cluster 2 center not found, present action aborted!")
                self._as.set_aborted()
                return

        timeout = rospy.Time.now() + rospy.Duration(10)

        # Wait until the part is no longer present in the gripper or timeout is reached
        while self._vacuum_state.part_present:
            rospy.loginfo("Part present, waiting...")
            if rospy.Time.now() > timeout:
                rospy.loginfo("Timeout reached, present action aborted!")
                self._as.set_aborted()
                return

            self._rate.sleep()

        rospy.loginfo(f"Turning off the vacuum")

        # Send a drop-off goal to the vacuum gripper
        goal = DropOffGoal(timeout=10)
        self._dropoff_client.send_goal(goal)

        # Return to the home position after drop-off is completed
        succeeded = self._group.go(start_pos, wait=True)
        if succeeded:
            rospy.loginfo(f"Returned to home position")
            return

        if not succeeded:
            rospy.loginfo(f"Could not return to home position")
            self._as.set_aborted()
            return

        rospy.loginfo(f"Part has been picked by customer")
        self._as.set_succeeded()

    def customer_detection(self):
        # Example logic for customer detection
        if self.customer_detected:
            rospy.loginfo("Customer detected in the frame")
            return True
        else:
            rospy.loginfo("No customer detected in the frame")
            return False

    def get_cluster_center_from_tf(self, frame_id):
        try:
            # Retrieve the cluster center position from the TF tree
            trans, _ = self._tl.lookupTransform("base_link", frame_id, rospy.Time())
            transform = geometry_msgs.msg.Transform()
            transform.translation.x = trans[0]
            transform.translation.y = trans[1]
            cluster_center = [transform.translation.x, transform.translation.y]
            return cluster_center
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Error retrieving cluster center from TF")
            return None

    def point_arm_to_cluster(self, cluster_center):
        joint_positions = self._group.get_current_joint_values()
        rospy.loginfo(f"Obtained the current joint positions")

        target_x = cluster_center[0]
        target_y = cluster_center[1]
        current_x = joint_positions[0]
        current_y = joint_positions[1]

        # Calculate the rotation angle to point the arm towards the cluster center
        rotation_angle = math.atan2(target_y - current_y, target_x - current_x)
        rospy.loginfo(f"Rotation angle calculated")

        # Adjust the joint positions to point the arm towards the cluster center
        joint_positions[0] += rotation_angle
        rospy.loginfo(f"Calculated rotation angle has been set as desired joint position")
        joint_positions[-2] += math.pi / 2
        rospy.loginfo(f"Calculated the desired joint angle of the before last link and set as desired joint position")

        # Set the joint value target and move the arm
        self._group.set_joint_value_target(joint_positions)
        succeeded = self._group.go(wait=True)
        if succeeded:
            rospy.loginfo(f"First joint and before last joint rotated with the calculated joint angles")
        if not succeeded:
            self._as.set_aborted()
            rospy.loginfo(f"Could not rotate the arm to the desired calculated angles")


if __name__ == "__main__":
    rospy.init_node("present_server")
    PresentActionServer(rospy.get_name())
    rospy.spin()

