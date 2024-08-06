#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from std_msgs.msg import Float32MultiArray

last_processed_time = None

def calculate_wall_dimensions(points):
    rospy.loginfo("Calculating wall dimensions")

    # Convert point cloud to numpy arrays
    x_coords = []
    z_coords = []

    for p in points:
        x_coords.append(p[0])
        z_coords.append(p[2])  # Assuming 2D LiDAR in x-z plane

    x_coords = np.array(x_coords)
    z_coords = np.array(z_coords)

    if len(x_coords) == 0 or len(z_coords) == 0:
        rospy.loginfo("No valid points found in the point cloud data")
        return

    width = max(x_coords) - min(x_coords)
    height = max(z_coords) - min(z_coords)

    # Publish wall dimensions
    if width is not None and height is not None:
        dimensions = Float32MultiArray(data=[width, height])
        wall_dimensions_pub.publish(dimensions)

    rospy.loginfo(f"Estimated Wall Width: {width} meters")
    rospy.loginfo(f"Estimated Wall Height: {height} meters")

def point_cloud_callback(data):
    global last_processed_time
    current_time = rospy.get_time()
    
    if last_processed_time is None or (current_time - last_processed_time) >= 0.5:
        rospy.loginfo("Point cloud data received")
        points = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        calculate_wall_dimensions(points)
        last_processed_time = current_time

def main():
    rospy.init_node('wall_measurement_node', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, point_cloud_callback)
    rospy.loginfo("Wall Measurement node initialized and subscribed to /velodyne_points")
    
    global wall_dimensions_pub
    wall_dimensions_pub = rospy.Publisher('/wall_dimensions', Float32MultiArray, queue_size=10)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
