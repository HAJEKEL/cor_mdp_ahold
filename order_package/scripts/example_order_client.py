#!/usr/bin/env python3

import rospy
from order_package.srv import AddTask
from order_package.msg import OrderGoal, OrderRequest

class OrderClient:
    def __init__(self):
        rospy.init_node('order_client')
        rospy.wait_for_service('order_node/add_task')
        self.order_service = rospy.ServiceProxy('order_node/add_task', AddTask)

    def send_order(self, order: OrderRequest):
        resp = self.order_service(order)
        return resp

if __name__ == '__main__':
    task = OrderRequest()
    rospy.loginfo("Sending order")
    task.product_name = "milk"
    task.request_type = 0
    task.order_id = 1

    client = OrderClient()
    resp = client.send_order(task)
    rospy.loginfo(f"Order sent: {resp}")

