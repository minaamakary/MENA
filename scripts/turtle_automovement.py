#!/usr/bin/env python
import rospy
import time
import math
import gc
import sys
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

odomdata = Odometry() #Global variable
checkPoints = True # Global variable to control movement
rotation = False # Global variable to control rotation
goalorientation = 0.0 # Global variable to store the goal orientation
def checkDistance(data):
    move = Twist()
    if (move.linear.x ==0 and move.angular.z ==0):
    # This function is triggered each time a new PointCloud2 message is received
        gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        min_distance = float('inf')
        for point in gen:
            x, y, z = point
            # Calculate the distance of the point from the LiDAR
            distance = np.sqrt(x**2 + y**2 + z**2)
            # Check if the point is directly in front of the LiDAR
            if abs(y) / np.sqrt(x**2 + z**2) < 0.1:  
                if distance < min_distance:
                    min_distance = distance
            del point
            del x, y, z 

        if min_distance < float('inf'):
            rospy.loginfo("Nearest object in front is at distance: {:.2f} meters".format(min_distance))
            
            obstacleavoid(min_distance, move)
        else:
            rospy.loginfo("No object detected directly in front.")
        del gen
        gc.collect()     


def turn_90_degrees(move_cmd, pub):
    global rotation
    global checkPoints
    global odomdata
    global goalorientation
    if rotation == False:
        print("Turning 90 degrees")
        rotation = True
        goalorientation = odomdata + 1.5708
        print(goalorientation)
        if goalorientation > math.pi:
            goalorientation = goalorientation - 2*math.pi
        print(goalorientation)
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.45
        pub.publish(move_cmd)

    else:
        if odomdata < goalorientation:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.45
            pub.publish(move_cmd)
            print("Already turning")
        else:
            rotation = False
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            print("Turned 90 degrees")
        
        
    

def obstacleavoid(distance, move):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Initialize the publisher
    global rotation
    if distance <= 0.75 or rotation == True:
        move.angular.z = 0.0
        pub.publish(move)
        turn_90_degrees(move, pub)


    else:
        move.linear.x = 0.35
        move.angular.z = 0.0    
        pub.publish(move)

def savingOdomData(data):
    global odomdata
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    
    odomdata = yaw


def main():
    rospy.Subscriber('/odom', Odometry, savingOdomData)  # Subscribe to the /odom topic
    rospy.init_node('robot_automove', anonymous=False)  # Initialize the node
    rospy.Subscriber('/velodyne_points', PointCloud2, checkDistance)  # Subscribe to the topic
    
    
    rospy.loginfo("Subscribed to /odom for pose information.")
    
    rate = rospy.Rate(50)

    print("Initialised Node Robot Automove ...")
    print("Subscribed to Velodyne Points")

    rospy.spin() # so that the code deesnt stop 
        
if __name__ == '__main__':
    main()


    #SUBSCRIBE TO pose
    #WHEN TURNING CHECK Z ROTATION DATA AND IDENTIFY THE GOAL ORIENTATION 
    #AND THEN LET IT ROTATE UNTIL IT REACHES THAT GOAL ORIENTATION WITH A FLAG 
#ISSUE 1 - IT DOESNT STOP BEFORE ROTATING 
#ISSUE 2 - THERE IS INACCURATE DATA WE NEED TO IMPLEMENT KALMAN FILTER 