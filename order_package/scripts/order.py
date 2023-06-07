#!/usr/bin/env python3

from order_client import OrderClient
from order_package.msg import OrderRequest
from customer_interface import CustomerInterface
import rospy
import pandas as pd

box_cap = 10

# Check order size
def Split_order(order_list):
    sub_orders = [[]]
    n_items = 0
    current_sub_order = 0
    for order in order_list:
        if n_items < box_cap:
            order.order_id += float(current_sub_order/10)
            sub_orders[current_sub_order].append(order)
            n_items += 1
        else:
            current_sub_order += 1
            n_items = 0
            sub_orders.append([])
    
    return sub_orders
            



if __name__ == '__main__':

    product_data_path = rospy.get_param("/product_param_loader/product_data_path")

    order_client = OrderClient()

    order_id = 0

    while not rospy.is_shutdown():

        customer_interface = CustomerInterface(product_data_path)
        item_amounts = customer_interface.run()
        order_requests = order_client.generate_orders(item_amounts, order_id)

        if not order_requests:
            continue

        sub_orders = Split_order(order_requests)

        for sub_order in sub_orders:
            for order in sub_order:
                order_client.send_order(order)

            # return to base station after sub order
            order = OrderRequest()
            order.product_name = "base_station"
            order.request_type = 3
            order.order_id = 100
            order_client.send_order(order)

        order_id += 1



