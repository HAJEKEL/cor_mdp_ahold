#!/usr/bin/env python3


import rospy
from order_package.msg import OrderGoal, OrderRequest
from order_package.srv import AddTask
from std_srvs.srv import Trigger, TriggerResponse
from request_to_goal import request_to_goal
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tabulate import tabulate

"""
This node will be resposible for handling the orders.
It recieves order requests from the user interface, or from customers in the store,
and maintains a list of orders to be processed.
It will act as a service and advertise the current product that is being processed.
"""

class OrderHandlerNode:
    def __init__(self):
        self.order_list = [] # List of orders to be processed

        self.order_list_table = "" 
        self.order_list_table_heavy = ""

        # the current order is the first order in the order list
        self.current_order = None # Order that is currently being processed

        self._request_srv = rospy.Service('/order_node/order_request', AddTask, self.handle_order_request)
        self._mark_completed_srv = rospy.Service('/order_node/mark_completed', Trigger, self.handle_mark_completed)
        self._current_order_pub = rospy.Publisher('/order_node/current_order', OrderGoal, queue_size=10)
        self._current_order_pub_string = rospy.Publisher('/order_node/current_order_string', String, queue_size=10)
        self._current_order_pub_waypoint = rospy.Publisher('/order_node/current_order_waypoint', Pose, queue_size=10)

        self._order_list_pub = rospy.Publisher('/order_node/order_list', String, queue_size=10)





    def handle_order_request(self, req):
        """
        This function will handle the order request.
        It will convert an order request to an order goal, and add it to the order list.
        """
        # Convert the order request to an order goal
        goal = request_to_goal(req.request)

        # Add the order goal to the order list
        if goal.request_type == 0: # this means the order is a pick in basket
            self.order_list.append(goal) # add product to the end of the list
            rospy.loginfo(f"Pick in basket order added to the end of the order list")

        elif goal.request_type in [1,2]: # this means the order must be picked or shown to the customer
            self.order_list.insert(0, goal) # add product to the front of the list
            rospy.loginfo(f"In-store customer order  added to the front of the order list")

        # Publish the order list
        self.update_order_list()

        return True


    def handle_mark_completed(self, req):
        """
        This function will handle the mark completed request.
        It will remove the order from the order list and add it to the completed list.
        """
        response = TriggerResponse()
        if not self.order_list:
            response.success = False
            response.message = "Order list is empty"
            return response

        self.order_list.remove(self.current_order)
        response.success = True
        response.message = f"Order {self.current_order.product_name} to {self.current_order.location_name} with request type {self.current_order.request_type} is removed from the order list"

        self.completed_list.append(self.current_order)

        self.update_order_list()

        return response


    def update_order_list(self):
        """
        This function will convert the order list to a table using tabulate and publish it to the order list topic.
        Because of the way ros strings work, we will publish the table line by line
        """
        table = []
        for order in self.order_list:
            table.append([order.product_name, order.location_name, order.order_id, order.request_type])
        self.order_list_table = tabulate(table, headers=['Product', 'Location', 'Order ID', 'Request type'], tablefmt='orgtbl')
        self.order_list_table_heavy = tabulate(table, headers=['Product', 'Location', 'Order ID', 'Request type'], tablefmt='heavy_grid')
        print(self.order_list_table_heavy)

        # Publish the order list line by line
        for line in self.order_list_table.split('\n'):
            self._order_list_pub.publish(line)




    def run(self):
        """
        This function will run the order handler.
        It will publish the current order to the current order topic.
        It will also publish the order list
        """



        if self.order_list:
            self.current_order = self.order_list[0]


        # Publish the current order
        if self.current_order is not None:
            current_order_string = f"Current order: {self.current_order.product_name} to {self.current_order.location_name} with request type {self.current_order.request_type}"
            self._current_order_pub.publish(self.current_order)
            self._current_order_pub_string.publish(current_order_string)
            self._current_order_pub_waypoint.publish(self.current_order.waypoint)





if __name__ == '__main__':
    rospy.init_node('order_handler')
    order_handler = OrderHandlerNode()
    while not rospy.is_shutdown():
        order_handler.run()
    rospy.spin()






