#!/usr/bin/env python3

import rospy
import actionlib
import sys
from voice_requests.msg import CustomerInteractionAction, CustomerInteractionGoal
from order_package.srv import AddTask
from order_package.msg import OrderRequest






class CustomerInteractionClient(object):
    def __init__(self):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient("/customer_interaction_server", CustomerInteractionAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

        # make a client for the order request server
        rospy.wait_for_service('order_node/add_task')
        self.order_service = rospy.ServiceProxy('order_node/add_task', AddTask)


    def run(self):
        # Send goal
        goal = CustomerInteractionGoal()
        self.client.send_goal(goal)
        # Get the result
        rospy.loginfo("Waiting for result...")
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Customer interaction failed")
            return

        rospy.loginfo("Customer interaction succeeded")
        result = self.client.get_result()

        # send order request
        request = OrderRequest()
        request.product_name = result.wanted_product

        if result.picking_assistance:
            request.request_type = 1
        else:
            request.request_type = 2
        
        request.order_id = 0

        rospy.loginfo("Sending order request...")

        response = self.order_service(request)
        if response.success:
            rospy.loginfo("Order request succeeded")
        else:
            rospy.loginfo("Order request failed")




if __name__ == "__main__":
    rospy.init_node("test")
    client = CustomerInteractionClient()
    client.run()
