#!/usr/bin/env python

import rospy
from human_detection.srv import LidarScanRegions

def lidar_service_client():
    rospy.wait_for_service('lidar_scan_regions')
    try:
        lidar_scan_regions = rospy.ServiceProxy('lidar_scan_regions', LidarScanRegions)
        response = lidar_scan_regions()

        # Process the received camera poses
        camera_poses = response.camera_poses

        # Use the received camera poses for further processing

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('lidar_service_client')
    lidar_service_client()
