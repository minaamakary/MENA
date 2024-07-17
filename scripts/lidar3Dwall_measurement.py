#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def calculate_wall_dimensions(ranges, angle_min, angle_increment):
    rospy.loginfo("Calculating wall dimensions")

    # Convert laser scan ranges to Cartesian coordinates
    angles = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)
    x_coords = ranges * np.cos(angles)
    z_coords = ranges * np.sin(angles)  # Assuming 2D LiDAR in x-z plane

    # Filter out infinite and invalid readings
    valid_indices = np.isfinite(x_coords) & np.isfinite(z_coords)
    x_coords = x_coords[valid_indices]
    z_coords = z_coords[valid_indices]

    if len(x_coords) == 0 or len(z_coords) == 0:
        rospy.loginfo("No valid points found in the laser scan data")
        return

    width = max(x_coords) - min(x_coords)
    height = max(z_coords) - min(z_coords)

    rospy.loginfo(f"Estimated Wall Width: {width} meters")
    rospy.loginfo(f"Estimated Wall Height: {height} meters")

def laser_scan_callback(data):
    rospy.loginfo("Laser scan data received")
    ranges = np.array(data.ranges)
    calculate_wall_dimensions(ranges, data.angle_min, data.angle_increment)

def main():
    rospy.init_node('wall_measurement_node', anonymous=True)
    rospy.Subscriber('/velodyne_points', LaserScan, laser_scan_callback)
    rospy.loginfo("Wall Measurement node initialized and subscribed to /velodyne_points")
   
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
