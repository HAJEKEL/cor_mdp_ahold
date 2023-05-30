#!/usr/bin/env python3

import rospy
import actionlib
from collision_box_interface import CollisionBoxInterface


if __name__ == "__main__":
    rospy.init_node("collision_box_test")
    interface = CollisionBoxInterface()
    rospy.sleep(1)
    interface.add_product_box()
    rospy.sleep(1)
    interface.attach_box()
    rospy.sleep(1)
    interface.add_basket_box()
    rospy.sleep(1)

    interface.detach_box()
    interface.remove_box()



