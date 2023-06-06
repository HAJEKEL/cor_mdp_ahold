#!/usr/bin/env python3

from order_client import OrderClient
from order_package.msg import OrderRequest
from customer_interface import CustomerInterface
import rospy
import pandas as pd




if __name__ == '__main__':

    product_data_path = rospy.get_param("/product_param_loader/product_data_path")

    order_client = OrderClient()

    order_id = 0

    while not rospy.is_shutdown():

        customer_interface = CustomerInterface(product_data_path)
        item_amounts = customer_interface.run()
        order_requests = order_client.generate_orders(item_amounts, order_id)
        for order in order_requests:
            order_client.send_order(order)


        # return to base station after order
        order = OrderRequest()
        order.product_name = "base_station"
        order.request_type = 3
        order.order_id = 100
        order_client.send_order(order)

        order_id += 1



