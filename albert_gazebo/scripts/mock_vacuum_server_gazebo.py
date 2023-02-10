#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
import tf

from franka_vacuum_gripper.msg import *

# Helpers
def _it(self):
    yield self.x
    yield self.y
    yield self.z


Point.__iter__ = _it


def _it(self):
    yield self.x
    yield self.y
    yield self.z
    yield self.w


Quaternion.__iter__ = _it


class Gazebo(object):
    def __init__(self, name: str) -> None:
        # Read config
        self.simulation = True  # rospy.get_param("simulation")

        # Initialize action server
        self._rate = rospy.Rate(20)
        self._action_name = name
        self._vacuum_as = actionlib.SimpleActionServer(
            "/franka_vacuum_gripper/vacuum",
            VacuumAction,
            execute_cb=self.vacuum_cb,
            auto_start=False,
        )
        self._dropoff_as = actionlib.SimpleActionServer(
            "/franka_vacuum_gripper/dropoff",
            DropOffAction,
            execute_cb=self.drop_off_cb,
            auto_start=False,
        )

        # Initialize vacuum gripper action client and related variable
        if not self.simulation:
            self._vacuum_client = actionlib.SimpleActionClient(
                "/franka_vacuum_gripper/vacuum", VacuumAction
            )
            rospy.loginfo(f"Waiting for vacuum server")
            self._vacuum_client.wait_for_server()
            rospy.loginfo(f"Vacuum server found")

            self._dropoff_client = actionlib.SimpleActionClient(
                "/franka_vacuum_gripper/dropoff", DropOffAction
            )
            rospy.loginfo(f"Waiting for dropoff server")
            self._dropoff_client.wait_for_server()
            rospy.loginfo(f"dropoff server found")

        self._vacuum_as.start()
        self._dropoff_as.start()

    def vacuum_cb(self, req):
        rospy.loginfo(f"Vacuum request received")

        pre_pose = PoseStamped()
        pre_pose.pose.position.z = 0.45
        pre_pose.pose.orientation.x = 1
        pre_pose.pose.orientation.w = 0
        pre_pose.header.frame_id = "tag_{}".format(req.goal_id)

        self._group.set_pose_target(pre_pose)
        succeeded = self._group.go(wait=True)
        rospy.loginfo(f"Pick action pre-pose {'succeeded' if succeeded else 'failed'}")

        # TODO: use linear path forward
        pose = PoseStamped()
        pose.pose.position.z = 0.33
        pose.pose.orientation.x = 1
        pose.pose.orientation.w = 0
        pose.header.frame_id = "tag_{}".format(req.goal_id)
        rospy.loginfo(f"Pick action pose {'succeeded' if succeeded else 'failed'}")

        self._group.set_pose_target(pose)
        succeeded = self._group.go(wait=True)

        # TODO: use linear path forward
        post_pose = PoseStamped()
        post_pose.pose.position.z = 0.45
        post_pose.pose.orientation.x = 1
        post_pose.pose.orientation.w = 0
        post_pose.header.frame_id = "tag_{}".format(req.goal_id)

        self._group.set_pose_target(post_pose)
        succeeded = self._group.go(wait=True)

        rospy.loginfo(f"Pick action post-pose {'succeeded' if succeeded else 'failed'}")

        if succeeded:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()

        return


if __name__ == "__main__":
    rospy.init_node("pick_server_moveit")
    PickActionServer(rospy.get_name())
    rospy.spin()
