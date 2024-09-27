#!/usr/bin/env python

import rospy
import open3d as o3d
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import numpy as np
from tf.transformations import euler_matrix, quaternion_matrix
from threading import Thread, Lock
import os
import sensor_msgs.point_cloud2 as pc2
import time  # Ensure time is imported

global point_cloud, robot_pose, robot_yaw, pcd_lock
point_cloud = o3d.geometry.PointCloud()
robot_pose = None
robot_yaw = None
pcd_lock = Lock()

def pose_callback(data):
    global robot_pose
    robot_pose = data.pose

def yaw_callback(data):
    global robot_yaw
    robot_yaw = data.data

def processed_point_cloud_callback(data):
    global point_cloud, robot_pose, robot_yaw, pcd_lock

    if robot_pose is None or robot_yaw is None:
        rospy.loginfo("Pose or yaw not available yet, skipping point cloud update.")
        return

    try:
        rospy.loginfo("Received processed point cloud data.")
        new_points = np.array(list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))
        new_point_cloud = o3d.geometry.PointCloud()
        new_point_cloud.points = o3d.utility.Vector3dVector(new_points)

        transformed_points = []
        for point in new_points:
            transformed_point = transform_point(point, robot_pose, robot_yaw)
            if transformed_point is not None:
                transformed_points.append(transformed_point)

        new_point_cloud.points = o3d.utility.Vector3dVector(np.array(transformed_points))
        with pcd_lock:
            point_cloud += new_point_cloud 

    except Exception as e:
        rospy.logerr(f"Error processing point cloud: {e}")

def transform_point(point, pose, yaw):
    try:
        robot_x = pose.position.x
        robot_y = pose.position.y
        robot_z = pose.position.z
        rotation_matrix = euler_matrix(0, 0, yaw)[:3, :3]
        transformed_point = np.dot(rotation_matrix, point) + [robot_x, robot_y, robot_z]
        return transformed_point

    except Exception as e:
        rospy.logerr(f"Error transforming point: {e}")
        return None

def visualize_point_cloud():
    global point_cloud, pcd_lock
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Real-Time PCD Visualization", width=1800, height=1800)
    is_added = False
    pcd_vis = o3d.geometry.PointCloud()

    while not rospy.is_shutdown():
        with pcd_lock:
            if not point_cloud.has_points():
                continue  

            # Copy the point cloud data safely
            pcd_vis.points = o3d.utility.Vector3dVector(np.asarray(point_cloud.points))

        if not is_added:
            vis.add_geometry(pcd_vis)
            is_added = True
        else:
            vis.update_geometry(pcd_vis)

        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.01)  # Optional: Add a small sleep to improve responsiveness

    vis.destroy_window()

def save_map(filename):
    full_path = filename  
    rospy.loginfo(f"Saving point cloud to {full_path}")
    with pcd_lock:
        if point_cloud and len(point_cloud.points) > 0:
            o3d.io.write_point_cloud(full_path, point_cloud)
            rospy.loginfo("Point cloud saved successfully!")
        else:
            rospy.logwarn("No point cloud data to save.")

def shutdown_hook():
    rospy.loginfo("Shutting down, saving map...")
    save_map("room_map.pcd")  

if __name__ == '__main__':
    rospy.init_node('room_mapper_node', anonymous=True)
    rospy.Subscriber('/robot_pose', geometry_msgs.PoseStamped, pose_callback)
    rospy.Subscriber('/robot_yaw', std_msgs.Float32, yaw_callback)
    rospy.Subscriber('/processed_point_cloud', sensor_msgs.PointCloud2, processed_point_cloud_callback)
    rospy.on_shutdown(shutdown_hook)
    vis_thread = Thread(target=visualize_point_cloud)
    vis_thread.start()

    rospy.spin()
