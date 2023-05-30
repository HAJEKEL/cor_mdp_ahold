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


class ScanShelf(object):
    def __init__(self, name):
        # Initialize action server
        self._rate = rospy.Rate(20)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ScanShelfAction,
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

        # Step 1: Joint goal start position
        rospy.loginfo('Pick action moving to start pos')
        start_pos = [goal.shelf_direction, -0.8, 0.0, -2.5, 0.0, 3.1, 0.8]
        self._group.go(start_pos, wait=True)
        rospy.loginfo('Pick action finished moving to start pos')

        look_up = start_pos.copy()
        look_up[5] += 0.3

        look_down = start_pos.copy()
        look_down[5] -= 0.6


        positions = [look_up, look_down, start_pos]

        # Pass through orientations
        for position in positions:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                return

            succeeded = self._group.go(position, wait=True)

        return


if __name__ == "__main__":
    rospy.init_node("scan_shelf_server")
    server = ScanShelf(rospy.get_name())
    rospy.spin()
