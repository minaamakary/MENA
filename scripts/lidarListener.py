#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import time

# Global variable to store the last processed time
last_processed_time = 0
# Interval in seconds
processing_interval = 3

# Define the room dimensions and positions of walls
# Assuming a rectangular room with walls parallel to the axes
room_length = 10.0  # length of the room
room_width = 8.0    # width of the room
room_height = 3.0   # height of the walls

# Define tolerance for detecting walls (in meters)
tolerance = 0.5

def lidar_callback(scan):
    global last_processed_time
    current_time = time.time()

    # Check if 3 seconds have passed since the last processing
    if current_time - last_processed_time >= processing_interval:
        # Update the last processed time
        last_processed_time = current_time

        # Extract relevant information from the scan data
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        # Convert polar coordinates to Cartesian coordinates
        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        # Initialize lists to hold points for each wall
        left_wall_points = []
        right_wall_points = []
        front_wall_points = []
        back_wall_points = []

        # Identify points for each wall
        for x, y in zip(x_coords, y_coords):
            # Left wall (assuming x is near 0)
            if abs(x) < tolerance:
                left_wall_points.append((x, y))
            # Right wall (assuming x is near room_length)
            elif abs(x - room_length) < tolerance:
                right_wall_points.append((x, y))
            # Front wall (assuming y is near room_width)
            elif abs(y - room_width) < tolerance:
                front_wall_points.append((x, y))
            # Back wall (assuming y is near 0)
            elif abs(y) < tolerance:
                back_wall_points.append((x, y))

        # Function to calculate width and height from points
        def calculate_dimensions(points):
            if not points:
                return 0, 0
            points = np.array(points)
            width = np.max(points[:, 0]) - np.min(points[:, 0])
            height = np.max(points[:, 1]) - np.min(points[:, 1])
            return width, height

        # Calculate dimensions for each wall
        left_wall_width, left_wall_height = calculate_dimensions(left_wall_points)
        right_wall_width, right_wall_height = calculate_dimensions(right_wall_points)
        front_wall_width, front_wall_height = calculate_dimensions(front_wall_points)
        back_wall_width, back_wall_height = calculate_dimensions(back_wall_points)

        # Print dimensions
        rospy.loginfo(f"Left Wall - Width: {left_wall_width:.2f} m, Height: {left_wall_height:.2f} m")
        rospy.loginfo(f"Right Wall - Width: {right_wall_width:.2f} m, Height: {right_wall_height:.2f} m")
        rospy.loginfo(f"Front Wall - Width: {front_wall_width:.2f} m, Height: {front_wall_height:.2f} m")
        rospy.loginfo(f"Back Wall - Width: {back_wall_width:.2f} m, Height: {back_wall_height:.2f} m")

def main():
    rospy.init_node('lidar_processor_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.loginfo("LiDAR Processor Node started, listening to /scan topic")
    rospy.spin()

if __name__ == '__main__':
    main()
