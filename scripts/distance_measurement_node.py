#!/usr/bin/env python

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Header
import open3d as o3d
import ros_numpy  # Ensure ros_numpy is installed
from threading import Lock
from std_msgs.msg import Float32  # To publish distance and use it in mappingRoom.py

# Kalman filter variables
kalman_state = 0.0  # Initial state (estimated distance)
kalman_covariance = 1.0  # Initial covariance
process_noise = 0.1  # Process noise (uncertainty in the model)
measurement_noise = 0.5  # Measurement noise (uncertainty in the measurements)

# Thread lock for Kalman filter variables
kalman_lock = Lock()

def kalman_filter(measurement, state, covariance, process_noise, measurement_noise):
    """
    Perform a Kalman filter update for a single scalar value.

    Args:
        measurement (float): The new measurement.
        state (float): The current state estimate.
        covariance (float): The current covariance estimate.
        process_noise (float): The process noise.
        measurement_noise (float): The measurement noise.

    Returns:
        tuple: Updated state and covariance.
    """
    # Predict
    predicted_state = state  # Assuming a constant model
    predicted_covariance = covariance + process_noise  # Increase uncertainty

    # Update
    kalman_gain = predicted_covariance / (predicted_covariance + measurement_noise)
    updated_state = predicted_state + kalman_gain * (measurement - predicted_state)
    updated_covariance = (1 - kalman_gain) * predicted_covariance

    return updated_state, updated_covariance

def point_cloud_callback(data):
    rospy.loginfo("Received point cloud data.")
    
    global kalman_state, kalman_covariance

    try:
        # Convert PointCloud2 message to a numpy structured array using ros_numpy
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        # Extract x, y, z coordinates
        x = pc_array['x']
        y = pc_array['y']
        z = pc_array['z']

        # Stack x, y, z into a (N, 3) array and remove NaNs
        points_array = np.stack([x, y, z], axis=-1)
        points_array = points_array[~np.isnan(points_array).any(axis=1)]

        if points_array.size == 0:
            rospy.logwarn("Point cloud is empty after removing NaNs.")
            return

        # Create an Open3D PointCloud object
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points_array)

        # ------------------- Step 1: PassThrough Filter -------------------
        # Define the region of interest (e.g., 0.5m to 5m in front of the robot)
        # Adjust the limits based on your simulation environment
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0.5, -2.0, 0.1),
                                                   max_bound=(5.0, 2.0, 3.0))
        filtered_pcd = point_cloud.crop(bbox)

        if len(filtered_pcd.points) == 0:
            rospy.logwarn("No points left after PassThrough filtering.")
            return

        # ------------------- Step 2: Plane Segmentation (RANSAC) -------------------
        # Segment the largest planar component (assumed to be the wall)
        plane_model, inliers = filtered_pcd.segment_plane(distance_threshold=0.02,
                                                           ransac_n=3,
                                                           num_iterations=1000)
        [a, b, c, d] = plane_model
        rospy.loginfo(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        if len(inliers) == 0:
            rospy.logwarn("Could not find a planar surface in the point cloud.")
            return

        # ------------------- Step 3: Compute Distance to Wall -------------------
        # The plane equation is ax + by + cz + d = 0
        # Distance from origin (robot's position) to the plane is |d| / sqrt(a^2 + b^2 + c^2)
        distance = abs(d) / np.sqrt(a**2 + b**2 + c**2)

        # Assuming the wall is in front of the robot (positive X direction)
        # You might need to adjust the sign based on the plane orientation
        # For example, if 'a' is positive, the wall is in front
        if a < 0:
            distance = -distance  # Wall is behind

        rospy.loginfo(f"Measured distance to wall: {distance:.2f} meters")

        # ------------------- Step 4: Apply Kalman Filter -------------------
        with kalman_lock:
            kalman_state, kalman_covariance = kalman_filter(
                measurement=distance,
                state=kalman_state,
                covariance=kalman_covariance,
                process_noise=process_noise,
                measurement_noise=measurement_noise
            )
            rospy.loginfo(f"Filtered distance to wall: {kalman_state:.2f} meters")

        # Optional: Publish the distance or perform additional actions
        # For example, publish to a custom topic or use it for navigation
        # Convert processed Open3D point cloud to ROS PointCloud2 format
        ros_cloud_msg = pc2.create_cloud_xyz32(data.header, np.asarray(filtered_pcd.points))
    
        # Publish the processed point cloud
        processed_point_cloud_pub.publish(ros_cloud_msg)

    except Exception as e:
        rospy.logerr(f"Error processing point cloud: {e}")


if __name__ == '__main__':
    rospy.init_node('wall_distance_node', anonymous=True)

    # Initialize global variables
    kalman_state = 0.0
    kalman_covariance = 1.0

    # Subscribe to the Velodyne point cloud topic
    point_cloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, point_cloud_callback)

    #publishing to topic filtered distance, to pick up the data in another script for mapping. I think we can remove filtered distance
    distance_pub = rospy.Publisher('/filtered_distance', Float32, queue_size=10) 
    processed_point_cloud_pub = rospy.Publisher('/processed_point_cloud', PointCloud2, queue_size=10)
    rospy.loginfo("Wall Distance Node Started.")
    rospy.spin()
