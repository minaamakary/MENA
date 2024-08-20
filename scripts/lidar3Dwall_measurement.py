#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

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

def scan_callback(data):
    
    print("scan callback")
    # Convert PointCloud2 to a list of tuples
    points = list(pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z")))
    # Convert to numpy array for easier processing
    cloud_array = np.array(points)
    print(points[0][0])
    
 
    #getDistance()


# x is forward -x means the obstacle is behind the lidar
# y is left right -y means the obstacle is on the right 
# z is up and down -z is 

        #twist.linear.x = 0.5
        #twist.angular.z = 0.0

    # move forward until  x <=0.5
    if (points[0][0]>0.5):
        while (points[0][0]>=0.5):
            twist.linear.x = 0.2
            #twist.angular.z = 0.0
    else:
        twist.angular.z = math.pi
        while (points[0][0]>=0.5):
            twist.linear.x = 0.2

    calculate_wall_dimensions(points)
    #point_cloud_callback(data)
    vel_publisher.publish(twist)


def is_robot_stopped():
        # Consider the robot stopped if both linear and angular velocities are near zero
    linear_stopped = (abs(twist.linear.x) < 0.01 and
                          abs(twist.linear.y) < 0.01 and
                          abs(twist.linear.z) < 0.01)

    angular_stopped = (abs(twist.angular.x) < 0.01 and
                           abs(twist.angular.y) < 0.01 and
                           abs(twist.angular.z) < 0.01)

    return linear_stopped and angular_stopped

twist = Twist()
min_distance = 0.5  # Desired distance to the wall
front_threshold = 1.0  # Distance to consider an obstacle in front

rospy.init_node('wall_measurement_node', anonymous=True)
            
try:
    print("TEST")
    rate = rospy.Rate(60)  # 10 Hz
    while not rospy.is_shutdown():
        print(is_robot_stopped())
        if is_robot_stopped():
            vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            current_velocity = 0
            #rospy.Subscriber('/odom', Odometry, odom_callback)   
            rospy.Subscriber('/velodyne_points', PointCloud2, scan_callback)
            rospy.loginfo("Wall Measurement node initialized and subscribed to /velodyne_points")
            global wall_dimensions_pub
            wall_dimensions_pub = rospy.Publisher('/wall_dimensions', Float32MultiArray, queue_size=10)

                # If the robot is stopped, process messages
            rospy.loginfo("Robot is stopped. Processing queued messages.")
        else:
            print("jhjhjhjh")
            rospy.loginfo("Robot is moving. Waiting to process messages.")
            # Sleep to maintain loop rate
        rate.sleep()

    #rospy.spin()

except rospy.ROSInterruptException:
    pass



