#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from human_detection.srv import ImageDetection
from geometry_msgs.msg import PoseStamped

def handle_image_detection(req):
    # Process the received image and camera pose
    image = req.image
    camera_pose = req.camera_pose

    # Perform human detection using the image and camera pose

    # Return the response indicating if humans were detected
    return True  # Replace with your detection result

def image_service_server():
    rospy.init_node('image_service_server')
    rospy.Service('image_detection', ImageDetection, handle_image_detection)
    rospy.spin()

if __name__ == '__main__':
    image_service_server()
