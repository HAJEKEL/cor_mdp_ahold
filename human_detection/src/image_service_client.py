#!/usr/bin/env python

import rospy
from human_detection.srv import ImageDetection
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

def image_service_client(image):
    rospy.wait_for_service('image_detection')
    try:
        image_detection = rospy.ServiceProxy('image_detection', ImageDetection)

        # Request the camera poses from the lidar service
        rospy.wait_for_service('lidar_scan_regions')
        lidar_scan_regions = rospy.ServiceProxy('lidar_scan_regions', LidarScanRegions)
        camera_poses = lidar_scan_regions()

        # Call the image detection service for each camera pose
        for camera_pose in camera_poses:
            response = image_detection(image, camera_pose)
            if response.humans_detected:
                return True

        return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('image_service_client')
    # Create an Image message and pass it to the client function
    image = Image()  # Replace with your actual image data
    result = image_service_client(image)
    if result:
        rospy.loginfo("Humans detected!")
    else:
        rospy.loginfo("No humans detected.")
