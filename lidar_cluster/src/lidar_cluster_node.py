#!/usr/bin/env python3.8

import rospy
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg

class PointCloudProcessor:
    def __init__(self):
        rospy.init_node('point_cloud_processor')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.point_cloud_subscriber = rospy.Subscriber('/point_cloud', PointCloud2, self.point_cloud_callback)
        self.tf_publisher = rospy.Publisher('/tf/frame_cluster', tf2_msgs.msg.TFMessage, queue_size=10)
        
    def point_cloud_callback(self, point_cloud):
        cluster_list, cluster_count = self.db_scan_point_cloud(point_cloud)
        customer_like_clusters = self.filter_customer_like_clusters(cluster_list)
        cluster_centers = self.calculate_cluster_centers(customer_like_clusters)
        self.publish_closest_cluster_frame(cluster_centers)
    
    def db_scan_point_cloud(self, point_cloud):
        db_scan = DBSCAN().fit(point_cloud)  # Use DBSCAN on the point cloud
        labels = db_scan.labels_
        cluster_count = len(np.unique(labels))  # Count the number of clusters found
        list_of_clusters = [point_cloud[labels == (i-1)] for i in range(cluster_count)]
        return list_of_clusters, cluster_count

    def filter_customer_like_clusters(self, cluster_list):
        customer_like_clusters = []
        
        for cluster in cluster_list:
            x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
            y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
            width = x_max - x_min
            depth = y_max - y_min
    
            if 0.2 < width < 1.2 and 0.2 < depth < 1.2:
                customer_like_clusters.append(cluster)
        
        return customer_like_clusters
    
    def calculate_cluster_centers(self, clusters):
        cluster_centers = []
    
        for cluster in clusters:
            x_min = np.min(cluster[:, 0])
            x_max = np.max(cluster[:, 0])
            y_min = np.min(cluster[:, 1])
            y_max = np.max(cluster[:, 1])
        
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
        
            center = [center_x, center_y]
            cluster_centers.append(center)
    
        return cluster_centers

    def publish_closest_cluster_frame(self, cluster_centers):
        # Get the closest cluster center (assuming only one cluster is considered here)
        closest_cluster_center = cluster_centers[0]
        
        # Get the transform from "base_link" to "frame_cluster_1"
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "frame_cluster_1", rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        
        # Create a TransformStamped message
        tf_msg = geometry_msgs.msg.TransformStamped()
        
        # Set the header of the message
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "frame_cluster_1"
        
        # Apply the transformation to the cluster center
        transformed_cluster_center = tf2_geometry_msgs.do_transform_point(closest_cluster_center, transform)
        
        # Set the translation values
        tf_msg.transform.translation.x = transformed_cluster_center.point.x
        tf_msg.transform.translation.y = transformed_cluster_center.point.y
        tf_msg.transform.translation.z = transformed_cluster_center.point.z
        
        # Set the rotation values (identity quaternion)
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        
        # Create a TFMessage and add the TransformStamped message to it
        tf_message = tf2_msgs.msg.TFMessage()
        tf_message.transforms.append(tf_msg)
        
        # Publish the TFMessage
        self.tf_publisher.publish(tf_message)

if __name__ == '__main__':
    try:
        point_cloud_processor = PointCloudProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
