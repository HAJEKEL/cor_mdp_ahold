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

# This function publishes the point cloud given as input on RViz. It is recommended to use it only in static situations
# where the environment does not change and the robot is stationary.
def publish_lidar_proposals(self, lidar_pedestrian_like):
    
    # Wait for a moment to allow roscore to establish connections
    rospy.sleep(1.)
    
    # Extract the cloud points
    cloud_points = lidar_pedestrian_like
    
    # Create a header for the message to be published
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map' 
    
    # Create a PointCloud2 message from the cloud points
    point_cloud_msg = pcl2.create_cloud_xyz32(header, cloud_points)
    
    # Publish the PointCloud2 message
    rospy.loginfo("Publishing lidar proposal point cloud!")
    self.point_cloud_pub.publish(point_cloud_msg)




def lidar_service_server():
    rospy.init_node('lidar_service_server')
    rospy.Service('lidar_scan_regions', LidarScanRegions, handle_lidar_scan)
    rospy.spin()

if __name__ == '__main__':
    lidar_service_server()

