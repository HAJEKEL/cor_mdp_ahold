#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker


class PlanArmIk(object):
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

    def run(self, pose_goal: Pose):
        """
        examples:
            pose_goal = geometry_msgs.msg.Pose() # Relative to frame /panda_link0!
            Example orientations: {x, y, z, w}

        """
        pose_goal_stamped = PoseStamped()
        pose_goal_stamped.header.frame_id = "panda_link0"
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

