#!/usr/bin/env python3

import rospy
from order_package.srv import AddTask
from order_package.msg import OrderGoal, OrderRequest
import tkinter as tk

def update_amount(item, change):
    amount = item_amounts.get(item)
    new_amount = amount + change

    if new_amount >= 0:
        item_amounts[item] = new_amount
        amount_label = amount_labels[item]
        amount_label.config(text=new_amount)

# Create a dictionary to store item amounts
item_amounts = {}

root = tk.Tk()
root.rowconfigure([0,1,2,3,4], minsize=50, weight=1)
root.columnconfigure([0, 1, 2, 3], minsize=50, weight=1)

order_list = ["Item 1", "Item 2", "Item 3", "Item 4"]

# Create labels to display the item names and amounts
amount_labels = {}
for i, item in enumerate(order_list):
    tk.Label(root, text='Enter amount of '+item+ ":").grid(row=i, column=0)
    amount_labels[item] = tk.Label(root, text="0")
    amount_labels[item].grid(row=i, column=2)

    # Initialize item amounts to 0
    item_amounts[item] = 0

    # Create buttons for increasing and decreasing the amount
    tk.Button(root, text="+", command=lambda item=item: update_amount(item, 1)).grid(row=i, column=3, sticky="nsew")
    tk.Button(root, text="-", command=lambda item=item: update_amount(item, -1)).grid(row=i, column=1, sticky="nsew")

root.mainloop()

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

