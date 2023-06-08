#!/usr/bin/env python3.8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class CustomerDetectionNode:
    def __init__(self):
        rospy.init_node("customer_detection_node")
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        rospy.Subscriber('/realsense_435/color/image_raw', Image, self.image_callback)
        self.customer_detected_publisher = rospy.Publisher('/customer_detected', Bool, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Apply the HOG detector to detect people
        boxes, weights = self.hog.detectMultiScale(cv_image, winStride=(4, 4), padding=(8, 8), scale=1.05)

        if len(boxes) > 0:
            #rospy.loginfo("Customer detected in the frame")
            customer_detected = True
        else:
            #rospy.loginfo("No customer detected in the frame")
            customer_detected = False

        self.customer_detected_publisher.publish(customer_detected)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = CustomerDetectionNode()
    node.run()


