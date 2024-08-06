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

class RoomMapper:
    def __init__(self):
        rospy.init_node('room_mapper_node', anonymous=True)

        self.point_cloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.accumulated_point_cloud_pub = rospy.Publisher('/accumulated_point_cloud', PointCloud2, queue_size=10)

        self.accumulated_points = []
        self.tf_listener = tf.TransformListener()
        self.robot_pose = None

        self.save_interval = rospy.Duration(10)  # Save the map every 10 seconds
        self.last_save_time = rospy.Time.now()

        rospy.loginfo("Room Mapper node initialized and subscribed to /velodyne_points and /odom")

    def odom_callback(self, data):
        # Store the latest robot pose
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        self.robot_pose = (position, orientation)
        rospy.loginfo(f"Updated robot pose: {self.robot_pose}")

    def transform_point(self, point, frame_id):
        point_stamped = PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = rospy.Time(0)
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]

        #try:
        #    transformed_point = self.tf_listener.transformPoint("map", point_stamped)
        return [point_stamped.point.x, point_stamped.point.y, point_stamped.point.z]
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #    rospy.logwarn(f"Transformation failed: {e}")
        #    return None

    def point_cloud_callback(self, data):
        if not self.robot_pose:
            rospy.logwarn("Robot pose not available yet.")
            return

        rospy.loginfo("Point cloud data received")
        points = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)

        point_count = 0
        for point in points:
            # Transform the point to the 'map' frame
            transformed_point = self.transform_point(point, data.header.frame_id)
            if transformed_point:
                self.accumulated_points.append(transformed_point)
                point_count += 1

        rospy.loginfo(f"Accumulated {point_count} new points. Total points: {len(self.accumulated_points)}")

        # Publish the accumulated point cloud
        if self.accumulated_point_cloud_pub.get_num_connections() > 0:
            self.publish_accumulated_point_cloud()

        # Periodically save the map
        current_time = rospy.Time.now()
        if current_time - self.last_save_time > self.save_interval:
            self.save_map('room_map.pcd')
            self.last_save_time = current_time

    def publish_accumulated_point_cloud(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"  # or the appropriate frame of reference

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        point_cloud = pc2.create_cloud(header, fields, self.accumulated_points)
        self.accumulated_point_cloud_pub.publish(point_cloud)

    def save_map(self, filename):
        # Define the full path to save the map file in the thesis directory
        full_path = f"{rospy.get_param('~map_save_path', '/home/minamakary/Desktop')}/{filename}"
        rospy.loginfo(f"Saving point cloud to {full_path}")

        if not self.accumulated_points:
            rospy.logwarn("No points to save!")
            return

        # Debugging: Print the number of points to be saved
        rospy.loginfo(f"Number of points to save: {len(self.accumulated_points)}")

        cloud = o3d.geometry.PointCloud()
        points_array = np.array(self.accumulated_points, dtype=np.float32)

        # Debugging: Print the shape of the points array
        rospy.loginfo(f"Points array shape: {points_array.shape}")

        if points_array.shape[1] != 3:
            rospy.logwarn("Points array does not have 3 columns!")
            return

        cloud.points = o3d.utility.Vector3dVector(points_array)
        o3d.io.write_point_cloud(full_path, cloud)
        rospy.loginfo("Point cloud saved successfully")

    def run(self):
        rospy.spin()
        self.save_map('room_map.pcd')

if __name__ == '__main__':
    mapper = RoomMapper()
    mapper.run()
