#!/usr/bin/env python3

"""
This helper function will take an order request and, if possible, 
convert it to an order goal using information on the parameter server.
"""

import rospy
from order_package.msg import OrderRequest, OrderGoal
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler



def waypoint_from_param(waypoint_name: str) -> Pose:
    """
    NOT USED ANYMORE: Converts a parameter server waypoint (x,y,z,yaw) name to a Pose object
    """
    result = Pose()

    # Get the waypoint from the parameter server
    dict_waypoint = rospy.get_param("/waypoints/"+ waypoint_name)

    result.position.x = float(dict_waypoint["x"])
    result.position.y = float(dict_waypoint["y"])
    result.position.z = float(dict_waypoint["z"])
    quaternion = quaternion_from_euler(0, 0, float(dict_waypoint["yaw"]))
    result.orientation.x = quaternion[0]
    result.orientation.y = quaternion[1]
    result.orientation.z = quaternion[2]
    result.orientation.w = quaternion[3]


    print("waypoint from param")
    print(result)
    rospy.loginfo(f"waypoint from param: {result}")


    return result



def request_to_goal(request: OrderRequest) -> OrderGoal:
    """
    Converts an OrderRequest to an OrderGoal if possible.
    """

    # Create the order goal
    goal = OrderGoal()
    goal.product_name = request.product_name
    goal.request_type = request.request_type
    goal.order_id = request.order_id

    # If the goal is to return to base, set destination to base_station
    if request.request_type == 3:
        goal.location_name = "base_station"
    else:
        goal.location_name = rospy.get_param("/products/" + request.product_name + "/store_location")

    #Get the waypoint for the store location from the parameter server
    try:
        waypoint_name = rospy.get_param("/store_locations/" + goal.location_name + "/waypoint")
    except KeyError:
        rospy.logerr("No waypoint for " + goal.location_name + " found on parameter server")

    # Get the waypoint from the parameter server
    dict_waypoint = rospy.get_param("/waypoints/"+ waypoint_name)

    goal.base_x = float(dict_waypoint["x"])
    goal.base_y = float(dict_waypoint["y"])
    goal.base_yaw = float(dict_waypoint["yaw"])




    # get the april tags for the store location from the parameter server
    if request.request_type != 3:
        try:
            goal.min_tag_id = rospy.get_param("/products/" + goal.product_name + "/min_tag_id")
            goal.max_tag_id = rospy.get_param("/products/" + goal.product_name + "/max_tag_id")
        except KeyError:
            rospy.logerr("No april tags for " + goal.product_name + " found on parameter server")

    return goal










