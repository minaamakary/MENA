#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

class WallMeasurement:
    def __init__(self):
        rospy.init_node('wall_measurement_node', anonymous=True)
        self.point_cloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback)
        self.point_cloud_data = None
        rospy.loginfo("WallMeasurement node initialized and subscribed to /velodyne_points")

    def point_cloud_callback(self, data):
        rospy.loginfo("Point cloud data received")
        self.point_cloud_data = data
        self.process_point_cloud()

    def process_point_cloud(self):
        if self.point_cloud_data is None:
            rospy.loginfo("No point cloud data available yet")
            return

        rospy.loginfo("Processing point cloud data")
        point_list = []
        for point in pc2.read_points(self.point_cloud_data, skip_nans=True):
            point_list.append([point[0], point[1], point[2]])

        points_np = np.array(point_list)
        rospy.loginfo(f"Number of points in cloud: {points_np.shape[0]}")
        self.calculate_wall_dimensions(points_np)

    def calculate_wall_dimensions(self, points):
        rospy.loginfo("Calculating wall dimensions")
        x_coords = points[:, 0]
        y_coords = points[:, 1]
        z_coords = points[:, 2]

        width = max(x_coords) - min(x_coords)
        height = max(z_coords) - min(z_coords)

        rospy.loginfo(f"Estimated Wall Width: {width} meters")
        rospy.loginfo(f"Estimated Wall Height: {height} meters")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    wall_measurement = WallMeasurement()
    try:
        wall_measurement.run()
    except rospy.ROSInterruptException:
        pass
