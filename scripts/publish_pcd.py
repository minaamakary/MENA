#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d

def publish_pcd_file(pcd_file_path):
    # Initialize the ROS node
    rospy.init_node('pcd_publisher', anonymous=True)
    pub = rospy.Publisher('/pcd_point_cloud', PointCloud2, queue_size=10)

    # Load the PCD file using Open3D
    pcd = o3d.io.read_point_cloud(pcd_file_path)
    points = np.asarray(pcd.points)

    # Define the header for the PointCloud2 message
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'  # or any frame you want to use

    # Create a PointCloud2 message from the PCD data
    pcd_msg = pc2.create_cloud_xyz32(header, points)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        pcd_msg.header.stamp = rospy.Time.now()  # Update timestamp
        pub.publish(pcd_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Replace with the path to your PCD file
        pcd_file_path = '/home/minamakary/Desktop/maps/room_map.pcd'
        publish_pcd_file(pcd_file_path)
    except rospy.ROSInterruptException:
        pass
