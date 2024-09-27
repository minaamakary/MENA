#!/usr/bin/env python3
 
import rospy
from math import radians
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
import numpy as np


SPEED = 0.3
MIN_DIST = 1.0
WALL_FOUND = False
width = 0
height = 0
left_scan = 0
right_scan = 0
front_scan = 0
x = 0
y = 0

def move_forward(pub, move):
	move.linear.x = SPEED
	pub.publish(move)
	 
def turn_clockwise(pub, move):
	move.linear.x = 0.0
	move.angular.z = -SPEED
	pub.publish(move)

def turn_anticlockwise(pub, move):
	move.linear.x = 0.0
	move.angular.z = SPEED
	pub.publish(move)
 
def scan_callback(data):
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
	move = Twist()
	front_scan = min(data.ranges[85:105])
	#print ("front scan -", front_scan)
	left_scan = min(data.ranges[165:175])
	#print ("left scan -", left_scan)
	right_scan = min(data.ranges[0:10])
	#print ("right scan -", right_scan)
	global WALL_FOUND 
	if (front_scan >= MIN_DIST and left_scan >= MIN_DIST and right_scan >= MIN_DIST and WALL_FOUND == False):
		move_forward(pub, move)
	 
	elif(front_scan <= MIN_DIST and left_scan <= MIN_DIST):
		turn_clockwise(pub,move)
	
	elif(left_scan <= MIN_DIST):
		move_forward(pub, move)

	elif(front_scan <= MIN_DIST ):
		WALL_FOUND = True
		turn_clockwise(pub,move)
		
	
		
	elif(left_scan >= MIN_DIST ):
		turn_anticlockwise(pub,move)
		move_forward(pub, move)

"""
def print_robot_pose(data):
    robot_name = "turtlebot3_waffle"
    try:
        index = data.name.index(robot_name)
        position = data.pose[index].position
        orientation = data.pose[index].orientation
		
        #print(f"Position: x={position.x}, y={position.y}, z={position.z}")
        #print(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
    except ValueError:
        print(f"Robot {robot_name} not found in the model states")

def calculate_wall_dimensions(data):
    wall_dimensions_pub = rospy.Publisher('/wall_dimensions', Float32MultiArray, queue_size=10)
    #rospy.loginfo("Calculating wall dimensions")
	
    points = list(pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z")))

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
    #draw(height,width)
	# Publish wall dimensions
    if width is not None and height is not None:
        dimensions = Float32MultiArray(data=[width, height])
        wall_dimensions_pub.publish(dimensions)

    #rospy.loginfo(f"Estimated Wall Width: {width} meters")
    #rospy.loginfo(f"Estimated Wall Height: {height} meters")
"""

def main ():
	#image = cv.imread('image.jpg', cv.IMREAD_GRAYSCALE)
	rospy.init_node('wall_follower', anonymous=False)
	global WALL_FOUND 
	WALL_FOUND = False
	# Subscriber to LaserScan data
	rospy.Subscriber('/scan', LaserScan, scan_callback) #change this to only /scan if you are using waffle and turtlebot3/scan if you are using burger
	#rospy.Subscriber('/velodyne_points', PointCloud2, calculate_wall_dimensions)
	#rospy.Subscriber("/gazebo/model_states", ModelStates, print_robot_pose)
	#print (x,y,front_scan,right_scan,left_scan,width,height)
	
	rate = rospy.Rate(10)
	rospy.spin()
   
	#rospy.signal_shutdown("shutting down ")
 
if __name__ == '__main__':
	main()
 
