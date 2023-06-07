#!/usr/bin/env python3

import rospy
from order_package.srv import AddTask
from order_package.msg import OrderGoal, OrderRequest
import tkinter as tk
import pandas as pd


class CustomerInterface:
    
    def __init__(self, product_db_path: str):
        # Read products from the csv file, and extract first column as product names
        self.product_db = pd.read_csv(product_db_path)
        self.product_names = self.product_db.iloc[:, 0].values

        # Create a dictionary to store the amount of each item with the item name as key
        self.root = tk.Tk()
        self.item_amounts = {product_name: tk.IntVar(value=0) for product_name in self.product_names}
        # Create the GUI
        self.draw()


    def update_amount(self, item, change):
        amount = self.item_amounts[item].get()
        new_amount = amount + change

        if new_amount >= 0:
            self.item_amounts[item].set(new_amount)


    def draw(self):
        # Create a label for the title
        for i, item in enumerate(self.item_amounts):
            tk.Label(self.root, text='enter amount of '+item+ ":").grid(row=i, column=0)
            
            # create a label for each item
            label = tk.Label(self.root, textvariable=self.item_amounts[item])
            label.grid(row=i, column=2)

            # create buttons for increasing and decreasing the amount
            tk.Button(self.root, text="+", command=lambda item=item: self.update_amount(item, 1)).grid(row=i, column=3, sticky="nsew")
            tk.Button(self.root, text="-", command=lambda item=item: self.update_amount(item, -1)).grid(row=i, column=1, sticky="nsew")
            exit_button = tk.Button(self.root, text="Submit Order", command=self.root.destroy)
            exit_button.grid(row=10, column=1, columnspan=3, sticky="nsew")

    def run(self):
        self.root.mainloop()
        return self.item_amounts




if __name__ == '__main__':
    product_data_path = rospy.get_param("/product_param_loader/product_data_path")

    customer_interface = CustomerInterface(product_data_path)
    customer_interface.root.mainloop()

