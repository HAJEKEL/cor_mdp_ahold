#!/usr/bin/env python

from sklearn.cluster import DBSCAN
import rospy
from sensor_msgs.msg import LaserScan
from human_detection.srv import LidarScanRegions
from geometry_msgs.msg import PoseStamped

def handle_lidar_scan(req):
    # Use planar lidar data to determine regions of interest where humans might be located

    # Calculate the proposed camera poses for each region of interest
    camera_poses = []
    for region in regions_of_interest:
        camera_pose = PoseStamped()
        # Calculate the pose for the camera to face the region
        # Add the calculated pose to the list of camera poses
        camera_poses.append(camera_pose)

    # Return the response with the proposed camera poses
    return camera_poses

def db_scan_point_cloud(self, point_cloud):
    db_scan = DBSCAN().fit(point_cloud)  # Use DBSCAN on the pointcloud
    labels = db_scan.labels_
    cluster_count = len(np.unique(labels))  # Counting the number of clusters found
    list_of_clusters = [point_cloud[labels == (i-1)] for i in range(cluster_count)]
    return list_of_clusters, cluster_count





def lidar_service_server():
    rospy.init_node('lidar_service_server')
    rospy.Service('lidar_scan_regions', LidarScanRegions, handle_lidar_scan)
    rospy.spin()

if __name__ == '__main__':
    lidar_service_server()

