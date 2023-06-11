#!/usr/bin/env python3.8

from sklearn.cluster import DBSCAN #sklearn installeren
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg
import tf2_geometry_msgs
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from sensor_msgs import point_cloud2



class ScanProcessor:
    def __init__(self):
        rospy.init_node('scan_processor')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.scan_subscriber = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.tf_publisher = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=10)
        
    def scan_callback(self, scan):
        # Convert LaserScan to PointCloud2
        points = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = 0.0  # Assuming a planar surface
            points.append([x, y, z])

        header = scan.header
        cloud_msg = create_cloud_xyz32(header, points)

        cluster_list, cluster_count = self.db_scan_scan(cloud_msg)
        customer_like_clusters = self.filter_customer_like_clusters(cluster_list)
        cluster_centers = self.calculate_cluster_centers(customer_like_clusters)
        self.publish_closest_cluster_frame(cluster_centers)
    
    def db_scan_scan(self, cloud_msg):
        points = np.array(list(point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"))))
        db_scan = DBSCAN().fit(points)  # Use DBSCAN on the point cloud
        labels = db_scan.labels_
        cluster_count = len(np.uniq[INFO] [1686474100.390857, 2251.041000]: No customer detected in cluster 1, checking cluster 2...
ue(labels))  # Count the number of clusters found
        list_of_clusters = [points[labels == (i-1)] for i in range(cluster_count)]
        return list_of_clusters, cluster_count


    def filter_customer_like_clusters(self, cluster_list):
        customer_like_clusters = []
        
        for cluster in cluster_list:
            x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
            y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
            width = x_max - x_min
            depth = y_max - y_min
    
            #if 0.2 < width < 1.2 and 0.2 < depth < 1.2:
            if 0 < width < 1.2 and 0 < depth < 1.2:

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

    def publish_closest_cluster_frame(self, cluster_centers, max_clusters=3):
        # Sort the cluster centers on distance, closest cluster gets number 1
        robot_position = [0, 0]  # Assuming the robot is at the origin [0, 0]
        cluster_centers = sorted(cluster_centers, key=lambda center: np.linalg.norm(np.array(center) - robot_position))

        # Create a TFMessage and add the TransformStamped message to it
        tf_message = tf2_msgs.msg.TFMessage()

        #maybe lidar_link
        # Get the transform from "base_link" to "frame_cluster_1"
        n_clusters=min(max_clusters, len(cluster_centers))
        for i in range(n_clusters):
            cluster_center = cluster_centers[i]


            # Create a TransformStamped message
            tf_msg = geometry_msgs.msg.TransformStamped()
            
            # Set the header of the message
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = "front_laser" #or in the lidar frame?
            tf_msg.child_frame_id = "frame_cluster_" + str(i+1)
            

            # Set the translation values
            tf_msg.transform.translation.x = cluster_center[0]
            tf_msg.transform.translation.y = cluster_center[1]
            tf_msg.transform.translation.z = 0
            
            # Set the rotation values (identity quaternion)
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 1.0

            tf_message.transforms.append(tf_msg)
        
        # Publish the TFMessage
        self.tf_publisher.publish(tf_message)



if __name__ == '__main__':
    try:
        scan_processor = ScanProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
