#!/usr/bin/env python3
import rospy
import actionlib
from retail_store_skills.msg import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from franka_vacuum_gripper.msg import *
from std_msgs.msg import Empty
from itertools import cycle
from tf.transformations import quaternion_from_euler
import moveit_commander


def _it(self):
    yield self.x
    yield self.y
    yield self.z
    yield self.w


Quaternion.__iter__ = _it


class LookForItem(object):
    def __init__(self, name):
        # Initialize action server
        self._rate = rospy.Rate(20)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            LookForItemAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )

        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity

        self.vacuum_state_subscriber = rospy.Subscriber(
            "/franka_vacuum_gripper/vacuum_state", VacuumState, self.vacuum_state_cb
        )

        self._as.start()

    def vacuum_state_cb(self, msg: VacuumState):
        self._vacuum_state = msg

    def execute_cb(self, goal):
        if self._vacuum_state.part_present:
            rospy.loginfo(f'Part already present, look around action aborted!')
            self._as.set_aborted()
            return

        try:
            product_loc_z = rospy.get_param("/" + goal.product_name + "/product_loc/z")
        except:
            rospy.loginfo("Product not found in knowledge base")
            rospy.loginfo("%s: Aborted" % self._action_name)
            self._as.set_aborted()
            return

        self._global_goal = PoseStamped()
        self._global_goal.header.frame_id = "panda_link0"
        self._global_goal.pose.position.z = product_loc_z - 0.6
        self._global_goal.pose.position.x = 0.5

        self._global_goal.pose.orientation = quaternion_from_euler(0, 0, 0)

        orientations = [
            Quaternion(*quaternion_from_euler(0, 1.57, 0)),
            Quaternion(*quaternion_from_euler(0.8, 1.57, 0)),
            Quaternion(*quaternion_from_euler(-0.8, 1.57, 0)),
        ]

        # Infinitely cycle through orientations
        for orientation in cycle(orientations):
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                return

            self._global_goal.pose.orientation = orientation

            self._group.set_pose_target(self._global_goal)
            succeeded = self._group.go(wait=True)


if __name__ == "__main__":
    rospy.init_node("look_for_item_server")
    server = LookForItem(rospy.get_name())
    rospy.spin()
