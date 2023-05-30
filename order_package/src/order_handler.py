#!/usr/bin/env python3
import math
import time
import rospy
from tabulate import tabulate
import tkinter as tk
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped

import pandas as pd
import os
import numpy as np

order_list = []
name_list = []

#parameters
box_cap = 40*30*30/2

# Specify the database file path (from cor_mdp_ahold directory)
file_path = 'order_package/src/database.ods'

# Import database file
df = pd.read_excel(file_path, header=0)
print(df)

def Set_parameters():
    for index, row in df.iterrows():
        name = str(np.array(df[['Name']])[index])
        # Removing the square brackets and single quotes
        name = name.strip("[]'")
        tag = float(np.array(df[['April_tag']])[index][0])
        x = float(np.array(df[['Location [x]']])[index][0])
        y = float(np.array(df[['Location [y]']])[index][0])
        yaw = float(np.array(df[['Orientation [yaw]']])[index][0])
        path = f'inventory/{name}'
        print(path)
        rospy.set_param(path, {'April_tag': tag, 'Location_x': x, 'Location_y': y, 'Orientation_yaw': yaw})

    #print(rospy.get_param('inventory/Tea_mango/April_tag'))

def Create_order():
    for index, row in df.iterrows():
        name = str(np.array(df[['Name']])[index])
        name = name.strip("[]'")
        print(name)
        GetInput(name)
        amount = int(user_input)
        for item in range(amount):
            tag = np.array(df[['April_tag']])[index][0]
            x = np.array(df[['Location [x]']])[index][0]
            y = np.array(df[['Location [y]']])[index][0]
            yaw = np.array(df[['Orientation [yaw]']])[index][0]
            size = np.array(df[['Size']])[index][0]
            order_list.append([name,tag,x,y,yaw,size])
        print(order_list)

# Check 
def Check_cap():
    global n_boxes
    order_size = 0
    #print(len(order_list))
    for index, product in enumerate(order_list):
        #print(product[4])
        order_size = order_size + product[5]
        if order_size >= box_cap:
            order_list.insert(index, 'init_loc')
            order_size = 0
    #print(len(order_list))

def Planner():
    for product in order_list:
        name_list.append(product[0])
    print(name_list)
    #publisher(.....)

def handle_input():
    global user_input
    user_input = entry.get()  # Get the input value
    print("User input:", user_input)
    window.destroy()  # Close the window

def GetInput(name):
    global window
    window = tk.Tk()

    # Create a label
    label = tk.Label(window, text='Enter amount of '+name+ ":")
    label.pack()

    # Create an entry field for input
    global entry
    entry = tk.Entry(window)
    entry.pack()

    # Create a button to handle the input
    button = tk.Button(window, text="Submit", command=handle_input)
    button.pack()

    window.mainloop()

def publisher(point, orien):
    rospy.init_node('order_handler', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rate = rospy.Rate(1) # Hz
    while not rospy.is_shutdown():
        p = PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = point[0]
        p.pose.position.y = point[1]
        p.pose.position.z = point[2]
        # Make sure the quaternion is valid and normalized
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0

        pub.publish(p)
        rate.sleep()
    
Set_parameters()
Create_order()
Check_cap()
Planner()