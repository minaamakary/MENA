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
 
