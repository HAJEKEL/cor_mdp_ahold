#!/usr/bin/env python3

import rospy
from order_package.srv import AddTask
from order_package.msg import OrderGoal, OrderRequest
import tkinter as tk
import pandas as pd



class OrderClient:
    def __init__(self):
        rospy.init_node('order_client')
        rospy.wait_for_service('order_node/add_task')
        self.order_service = rospy.ServiceProxy('order_node/add_task', AddTask)

    def generate_orders(self, item_amounts: dict, order_id: int):
        order_requests = []
        for item in item_amounts:
            amount = item_amounts[item].get()
            if amount > 0:
                for _ in range(amount): 
                    task = OrderRequest()
                    task.product_name = item
                    task.order_id = order_id
                    order_requests.append(task)

        return order_requests


    def send_order(self, order: OrderRequest):
        resp = self.order_service(order)
        return resp
