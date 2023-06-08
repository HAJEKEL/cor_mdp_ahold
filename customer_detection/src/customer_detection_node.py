#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from retail_store_skills.msg import CustomerDetectionAction, CustomerDetectionFeedback, CustomerDetectionResult


class CustomerDetectionNode:
    def __init__(self):
        rospy.init_node("customer_detection_node")
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        rospy.Subscriber('/realsense_435/color/image_raw', Image, self.image_callback)
        self.action_server = rospy.ActionServer(
            'customer_detection_action', CustomerDetectionAction, self.handle_customer_detection_action
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Apply the HOG detector to detect people
        boxes, weights = self.hog.detectMultiScale(cv_image, winStride=(4, 4), padding=(8, 8), scale=1.05)

        if len(boxes) > 0:
            rospy.loginfo("Customer detected in the frame")
            feedback = CustomerDetectionFeedback(customer_detected=True)
        else:
            rospy.loginfo("No customer detected in the frame")
            feedback = CustomerDetectionFeedback(customer_detected=False)

        self.action_server.publish_feedback(feedback)

    def handle_customer_detection_action(self, goal):
        # This function is called when a new customer detection action goal is received
        # You can perform any necessary processing here
        # For example, you can set a flag to enable/disable customer detection based on the goal

        # After processing, you can send the result
        result = CustomerDetectionResult(result_message="Customer detection action completed")
        self.action_server.set_succeeded(result)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = CustomerDetectionNode()
    node.run()
