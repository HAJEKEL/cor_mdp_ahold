#!/usr/bin/env python3

import rospy
import ast
import actionlib
from retail_store_skills.msg import *
import moveit_commander
import tf

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class HomeArmServer(object):
    """
    HomeArmServer is a ROS node that provides a service to home the arm to a base position.
    The base position should be a position such that the arm is positioned above the base, 
    and the robot will not unintentionally collide with customers
    """
    def __init__(self, name):
        self.simulation = True 
        #rospy.get_param("simulation")

        # Initialize action server
        self._rate = rospy.Rate(20)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, HomeArmAction, execute_cb=self.as_cb, auto_start=False)

        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("panda_arm")
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity
        self._tl = tf.TransformListener()

        # Get the arm home position from the parameter server (if available)
        if rospy.has_param("arm_home_position"):
            self._home_position = ast.literal_eval(rospy.get_param("arm_home_position"))

        else:
            rospy.logwarn("No arm home position found on parameter server, using default")
            self._home_position = [0.0, -0.8, 0.0, -2.5, 0.0, 3.1, 0.8]

        self._as.start()


    def as_cb(self, req):
        rospy.loginfo(f'Home arm action called')


        # Return a success response
        if self._as.is_preempt_requested():
            self._as.set_preempted()
        else:
            self._group.go(self._home_position, wait=True)
            self._as.set_succeeded()

        return



if __name__ == '__main__':
    rospy.init_node('home_arm_server')
    server = HomeArmServer(rospy.get_name())
    rospy.spin()




