#!/usr/bin/env python3

# this node will add a collision box to the robot to symbolize the basket

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, CollisionObject
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
import sys
import copy

class CollisionBoxInterface(object):
    def __init__(self):
        self._scene = moveit_commander.PlanningSceneInterface()
        self._robot = moveit_commander.RobotCommander()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")

    def wait_for_state_update(
            self, box_is_known=False, box_is_attached=False, timeout=4
            , box_name = None):
            start = rospy.get_time()
            seconds = rospy.get_time()
            while (seconds - start < timeout) and not rospy.is_shutdown():
                # Test if the box is in attached objects
                attached_objects = self._scene.get_attached_objects([box_name])
                is_attached = len(attached_objects.keys()) > 0

                # Test if the box is in the scene.
                # Note that attaching the box will remove it from known_objects
                is_known = box_name in self._scene.get_known_object_names()

                # Test if we are in the expected state
                if (box_is_attached == is_attached) and (box_is_known == is_known):
                    return True

                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
            return False

    def add_product_box(self, timeout=4, box_name="product_box"):

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_vacuum"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.05  # above the panda_vacuum frame
        self._scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        rospy.loginfo(f'Added product box to scene')

        return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name=box_name)


    def add_basket_box(self, timeout=4, box_name="basket_box"):

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = -0.45
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.15

        self._scene.add_box(box_name, box_pose, size=(0.3, 0.5, 0.3))
        rospy.loginfo(f'Added basket box to scene')
        return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name=box_name)

    def add_shelf_collision_boxes(self, tag_id: int, timeout=4):
        box_size = 0.5
        horizontal_offset = 0.2 + box_size/2
        vertical_offset = 0.3 + box_size/2

        frame_id = "tag_{}".format(tag_id)
        box_orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

        left_box_pose = PoseStamped()
        left_box_pose.header.frame_id = frame_id
        left_box_pose.pose.position.x = -horizontal_offset
        left_box_pose.pose.orientation = box_orientation
        self._scene.add_box("shelf_left", left_box_pose, size=(box_size, box_size, 0.2))
        self.wait_for_state_update(box_is_known=True, box_name="shelf_left")

        right_box_pose = PoseStamped()
        right_box_pose.header.frame_id = frame_id
        right_box_pose.pose.position.x = horizontal_offset
        right_box_pose.pose.orientation = box_orientation
        self._scene.add_box("shelf_right", right_box_pose, size=(box_size, box_size, 0.2))
        self.wait_for_state_update(box_is_known=True, box_name="shelf_right")

        top_box_pose = PoseStamped()
        top_box_pose.header.frame_id = frame_id
        top_box_pose.pose.position.y = vertical_offset
        top_box_pose.pose.orientation = box_orientation
        self._scene.add_box("shelf_top", top_box_pose, size=(box_size, box_size, 0.2))
        self.wait_for_state_update(box_is_known=True, box_name="shelf_top")

        bottom_box_pose = PoseStamped()
        bottom_box_pose.header.frame_id = frame_id
        bottom_box_pose.pose.position.y = -vertical_offset
        bottom_box_pose.pose.orientation = box_orientation
        self._scene.add_box("shelf_bottom", bottom_box_pose, size=(box_size, box_size, 0.2))
        self.wait_for_state_update(box_is_known=True, box_name="shelf_bottom")



    def remove_shelf_collision_boxes(self, timeout=4):
        self._scene.remove_world_object("shelf_left")
        assert self.wait_for_state_update(box_is_attached=False, box_is_known=False, box_name="shelf_left")
        self._scene.remove_world_object("shelf_right")
        assert self.wait_for_state_update(box_is_attached=False, box_is_known=False, box_name="shelf_right")
        self._scene.remove_world_object("shelf_top")
        assert self.wait_for_state_update(box_is_attached=False, box_is_known=False, box_name="shelf_top")
        self._scene.remove_world_object("shelf_bottom")
        assert self.wait_for_state_update(box_is_attached=False, box_is_known=False, box_name="shelf_bottom")


    def attach_box(self, timeout=4, box_name="product_box", touch_links=["panda_vacuum"]):
        eef_link = self._group.get_end_effector_link()

        self._scene.attach_box(eef_link, box_name, touch_links=touch_links)
        rospy.loginfo(f'Attached product box to panda_vacuum')

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
                box_is_attached=True, box_is_known=False, timeout=timeout, box_name=box_name
                )

    def detach_box(self, timeout=4, box_name="product_box"):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        eef_link = self._group.get_end_effector_link()

        self._scene.remove_attached_object(eef_link, name=box_name)
        rospy.loginfo(f'Detached product box from panda_vacuum')

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout, box_name=box_name
        )

    def remove_box(self, timeout=4, box_name="product_box"):

        self._scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        rospy.loginfo(f'Removed product box from scene')

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout, box_name=box_name
        )

