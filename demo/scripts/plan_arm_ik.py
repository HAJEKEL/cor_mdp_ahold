#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from franka_vacuum_gripper.msg import *
from geometry_msgs.msg import PoseStamped, Pose, Point
import tf_conversions
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker

from math import pi, tau, dist, fabs, cos



class PlanArmIK(object):
    def __init__(self, vis_pub):
        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()

        self.group = moveit_commander.MoveGroupCommander("panda_arm")

        # We can get the name of the reference frame for this robot:
        # Gripper pose will always be relative to this frame!
        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Initialize vacuum gripper action client and related variable
        self._vacuum_client = actionlib.SimpleActionClient("/franka_vacuum_gripper/vacuum", VacuumAction)
        rospy.loginfo(f'Waiting for vacuum server')
        self._vacuum_client.wait_for_server()
        rospy.loginfo(f'Vacuum server found')

        self._dropoff_client = actionlib.SimpleActionClient("/franka_vacuum_gripper/dropoff", DropOffAction)
        rospy.loginfo(f"Waiting for dropoff server")
        self._dropoff_client.wait_for_server()
        rospy.loginfo(f"Dropoff server found")


        self._vis_pub = vis_pub

    def publish_vis_marker(self, pose: PoseStamped):
        # Method to display a marker in RViz
        marker = Marker()
        marker.header.frame_id = pose.header.frame_id 
        marker.header.stamp = rospy.Time()
        marker.ns = "marker_vis"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.points = [Point(x=0, y=0, z=0), Point(x=0, y=0, z=0.1)]
        marker.scale.x = 0.01
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        rospy.loginfo("publishing marker")
        self._vis_pub.publish(marker)

    def activate_vacuum(self):
        goal = VacuumGoal(vacuum=10)
        self._vacuum_client.send_goal(goal)

    def deactivate_vacuum(self):
        goal = DropOffGoal()
        self._dropoff_client.send_goal(goal)


    def run(self, pose_goal: Pose, frame_id: str = "panda_link0"):
        """
        examples:
            pose_goal = geometry_msgs.msg.Pose() # Relative to frame /panda_link0!
            Example orientations: {x, y, z, w}

        """
        pose_goal_stamped = PoseStamped()
        pose_goal_stamped.header.frame_id = frame_id
        pose_goal_stamped.pose = pose_goal

        if self._vis_pub:
            self.publish_vis_marker(pose_goal_stamped)
        self.group.set_pose_target(pose_goal_stamped)

        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        if plan:
            self.group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.group.clear_pose_targets()
            rospy.loginfo("Goal reached")
            return True
        else:
            rospy.logfatal("Failed to find path..")
            return False

    def home(self):
        rospy.loginfo("Moving to home position")
        start_pos = [0.0, -0.8, 0.0, -2.5, 0.0, 3.1, 0.8]
        self.group.go(start_pos, wait=True)
        rospy.loginfo("Home position reached")

        rospy.loginfo("Vacuuming")


    def make_arm_goal_pose(self, ee_pose, vacuum_orientation=None, frame_id="panda_link0"):
        """
        creates a goal pose from a dictionary with position (and orientation)
        ee_pose: dict with keys "position" and "orientation"
        vacuum_orientation: "forward" or "down"
        """
        goal_pose = geometry_msgs.msg.Pose()

        goal_pose.position.x = ee_pose["position"]["x"]
        goal_pose.position.y = ee_pose["position"]["y"]
        goal_pose.position.z = ee_pose["position"]["z"]
        goal_pose.orientation.x = ee_pose["orientation"]["x"]
        goal_pose.orientation.y = ee_pose["orientation"]["y"]
        goal_pose.orientation.z = ee_pose["orientation"]["z"]
        goal_pose.orientation.w = ee_pose["orientation"]["w"]

        if vacuum_orientation is not None:
            if vacuum_orientation == "forward":
                # make the z axis point forward
                goal_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,tau/4, 0))
            elif vacuum_orientation == "down":
                # make the z axis point down
                goal_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, -tau/4))

        return goal_pose
    
    def approach_path(pose):
        waypoints = []
            
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


