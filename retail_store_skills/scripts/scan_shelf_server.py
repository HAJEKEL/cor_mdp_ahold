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
import tf
import numpy as np


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

        # Transform listener to look for april tags
        self._tl = tf.TransformListener()

        self._as.start()


    def vacuum_state_cb(self, msg: VacuumState):
        self._vacuum_state = msg

    def execute_cb(self, goal: ScanShelfGoal):
        if self._vacuum_state.part_present:
            rospy.loginfo(f'Part already present, look around action aborted!')
            self._as.set_aborted()
            return

        result = ScanShelfResult()

        # Step 1: Move arm to start position
        start_pos = [goal.shelf_direction, -0.8, 0.0, -2.5, 0.0, 3.1, 0.8]
        self._group.go(start_pos, wait=True)
        rospy.loginfo(f'Arm moved to start position')

        # Step 2: Look around
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
            self._group.go(position, wait=True)


        # Step 3: Check if there are any of the april tags that we want
        frames = self._tl.getFrameStrings()
        wanted_tags = range(goal.min_tag_id, goal.max_tag_id + 1)

        rospy.loginfo(f"Looking for tags {wanted_tags}")

        def get_tag_id(frame):
            return int(frame.replace("tag_", ""))

        found_tags = [(frame, get_tag_id(frame)) for frame in frames if "tag_" in frame and get_tag_id(frame) in wanted_tags]

        if not found_tags:
            rospy.loginfo(f"No april tags found!")
            self._as.set_aborted(result)
            return result

        rospy.loginfo(f"Found tags: {found_tags}")

        # Step 4: get the tag which is closest to the vacuum gripper frame
        def get_distance_to_gripper(tag):
            trans, _ = self._tl.lookupTransform(
                "panda_vacuum", tag, rospy.Time(0)
            )
            return np.linalg.norm(trans)

        closest_tag = min(found_tags, key=lambda tag: get_distance_to_gripper(tag[0]))
        rospy.loginfo(f"Closest tag is tag {closest_tag[1]}")

        # add the closest tag to the result
        result.tag_id = closest_tag[1]
        self._as.set_succeeded(result)

        return result












        

        

        return


if __name__ == "__main__":
    rospy.init_node("scan_shelf_server")
    server = ScanShelf(rospy.get_name())
    rospy.spin()
