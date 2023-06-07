#!/usr/bin/env python

import rospy
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import numpy as np
from sklearn.cluster import DBSCAN

class LidarClusterNode:
    def __init__(self):
        rospy.init_node('lidar_cluster_node')
        
        # Define ROS publishers and subscribers
        self.point_cloud_pub = rospy.Publisher('/lidar/point_cloud', pcl2.PointCloud2, queue_size=10)
        self.point_cloud_sub = rospy.Subscriber('/lidar/point_cloud', pcl2.PointCloud2, self.point_cloud_callback)
    
    def point_cloud_callback(self, point_cloud_msg):
        # Convert the incoming PointCloud2 message to a numpy array
        point_cloud = np.array(list(pcl2.read_points(point_cloud_msg, field_names=('x', 'y', 'z'))))
        
        # Perform DBSCAN clustering on the point cloud
        clusters, cluster_count = self.db_scan_point_cloud(point_cloud)
        
        # Filter pedestrian-like clusters
        pedestrian_like_clusters = self.filter_pedestrian_like_clusters(clusters)
        
        # Publish the pedestrian-like clusters as a point cloud on RViz
        self.publish_lidar_proposals(pedestrian_like_clusters)
        
    def db_scan_point_cloud(self, point_cloud):
        db_scan = DBSCAN().fit(point_cloud)  # Use DBSCAN on the point cloud
        labels = db_scan.labels_
        cluster_count = len(np.unique(labels))  # Count the number of clusters found
        list_of_clusters = [point_cloud[labels == (i-1)] for i in range(cluster_count)]
        return list_of_clusters, cluster_count

    def filter_pedestrian_like_clusters(self, cluster_list):
        pedestrian_like_clusters = []
        
        for cluster in cluster_list:
            x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
            y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
            width = x_max - x_min
            depth = y_max - y_min
    
            if 0.2 < width < 1.2 and 0.2 < depth < 1.2:
                pedestrian_like_clusters.append(cluster)
        
        return pedestrian_like_clusters

    def publish_lidar_proposals(self, lidar_pedestrian_like):
        # Create a header for the PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map' 
        
        # Create a PointCloud2 message from the pedestrian-like clusters
        point_cloud_msg = pcl2.create_cloud_xyz32(header, lidar_pedestrian_like)
        
        # Publish the PointCloud2 message
        rospy.loginfo("Publishing lidar proposal point cloud!")
        self.point_cloud_pub.publish(point_cloud_msg)

if __name__ == '__main__':
    try:
        node = LidarClusterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
