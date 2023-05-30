#!/usr/bin/env python3

import rospy
import actionlib

from closest_link_attacher.srv import Attach, AttachRequest, AttachResponse, Detach, DetachRequest

from franka_vacuum_gripper.msg import *

class FakeFrankaVacuumGripper(object):
    def __init__(self, name: str) -> None:
        # Initialize action server
        self._rate = rospy.Rate(20)
        self._vacuum_as = actionlib.SimpleActionServer(
            "~vacuum", VacuumAction, execute_cb=self.vacuum_cb, auto_start=False
        )
        self._dropoff_as = actionlib.SimpleActionServer(
            "~dropoff", DropOffAction, execute_cb=self.dropoff_cb, auto_start=False
        )

        self._state_publisher = rospy.Publisher('~vacuum_state', VacuumState, queue_size=10)

        # attach and detach servers
        rospy.loginfo(f"Looking for attach server")
        self._attach_srv = rospy.ServiceProxy(
            "/gazebo/closest_link_attacher/attach", Attach
        )
        self._attach_srv.wait_for_service()
        rospy.loginfo(f"Attach server found")

        rospy.loginfo(f"Looking for detach server")
        self._detach_srv = rospy.ServiceProxy(
            "/gazebo/closest_link_attacher/detach", Detach
        )
        self._detach_srv.wait_for_service()
        rospy.loginfo(f"Detach server found")

        self._vacuum_as.start()
        self._dropoff_as.start()

        self.state = VacuumState(part_present=False)

    def vacuum_cb(self, msg):
        attach_req = AttachRequest()
        attach_req.model = "panda"
        attach_req.link = "panda_vacuum"
        # if request does not succeed in 2 seconds, abort
        # record current time
        start_time = rospy.Time.now()
        while not self.state.part_present and (rospy.Time.now() - start_time).to_sec() < 2:
            try:
                res = self._attach_srv.call(attach_req)
                self.state.part_present = res.ok
                self._vacuum_as.set_succeeded()
            except:
                rospy.logwarn("attaching not succeeded")
                self._vacuum_as.set_aborted()
        

    def dropoff_cb(self, msg):
        detach_req = DetachRequest()
        self._detach_srv.call(detach_req)
        self.state.part_present = False
        self._dropoff_as.set_succeeded()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._state_publisher.publish(self.state)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("franka_vacuum_gripper")
    n = FakeFrankaVacuumGripper(rospy.get_name())
    n.run()
