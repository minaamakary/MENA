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
from math import radians

odomdata = Pose() #Global variable
checkPoints = True # Global variable to control movement



def turn_90_degrees(move_cmd, pub):
    #rospy.init_node('turn_90_degrees', anonymous=False)
    
    #pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the angular velocity
    angular_speed = 0.5  # radians per second
    turn_angle = math.pi / 2  # 90 degrees in radians
    
    # Calculate the time required to turn 90 degrees
    duration = turn_angle / angular_speed  # seconds
    
    move_cmd.linear.x = 0.0  # No forward movement
    move_cmd.angular.z = angular_speed  # Set the angular speed
    
    # Record the start time
    start_time = rospy.Time.now().to_sec()
    
    # Keep rotating until the duration is reached
    while rospy.Time.now().to_sec() - start_time < duration:
        print (rospy.Time.now().to_sec() - start_time, duration)
        print(start_time,duration,rospy.Time.now().to_sec())
        pub.publish(move_cmd)
        rospy.sleep(0.01)
    print ("hello") 
    # Stop the robot after the turn
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    



def turn_90_degrees_clockwise(move,pub):
    
    # Set the rate at which to send commands
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message and set the angular velocity
    move.linear.x = 0.0
    move.linear.y = 0.0
    move.linear.z = 0.0
    move.angular.x = 0.0
    move.angular.y = 0.0
    move.angular.z = -radians(45)  # Negative for clockwise (right) rotation
    
    # Calculate the duration needed to turn 90 degrees at the given speed
    # Since we are turning at 45 degrees per second, we need 2 seconds to turn 90 degrees
    duration = rospy.Duration(2)  # Duration in seconds
    
    # Record the start time
    start_time = rospy.Time.now()
    
    # Continue turning until the desired duration has passed
    while rospy.Time.now() - start_time < duration:
        pub.publish(move)
        rate.sleep()
    
def move_forward(x, y, z, move,pub):
    rate = rospy.Rate(10)  # 10 Hz
    distance = abs(x)
    #math.sqrt(x**2 + y**2 + z**2)
    move.linear.x = 0.2  # Set the forward speed
    move.angular.z = 0.0  # No rotation
   
    # Calculate the time needed to cover the distance
    duration = rospy.Duration(distance / 0.2)
    
    #Record the start time
    start_time = rospy.Time.now()
    
    # Move the robot forward until the desired distance is covered
    while rospy.Time.now() - start_time < duration:
        pub.publish(move)
        print(x,duration,distance)
        rate.sleep()
    
    # Stop the robot after moving the specified distance
    move.linear.x = 0.0
    pub.publish(move)
    rospy.loginfo(f"Moved forward {distance} meters")
    




def turn_90_degrees_anticlockwise(move,pub):
    
 
    # Set the rate at which to send commands
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message and set the angular velocity
   
    move.linear.x = 0.0
    move.linear.y = 0.0
    move.linear.z = 0.0
    move.angular.x = 0.0
    move.angular.y = 0.0
    move.angular.z = radians(45)  # Positive for anticlockwise (left) rotation
    
    # Calculate the duration needed to turn 90 degrees at the given speed
    # Since we are turning at 45 degrees per second, we need 2 seconds to turn 90 degrees
    duration = rospy.Duration(2)  # Duration in seconds
    
    # Record the start time
    start_time = rospy.Time.now()
    
    # Continue turning until the desired duration has passed
    while rospy.Time.now() - start_time < duration:
        pub.publish(move)
        rate.sleep()
    

def turn_180_degrees(move,pub):
        
    # Set the rate at which to send commands
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message and set the angular velocity
    move.linear.x = 0.0
    move.linear.y = 0.0
    move.linear.z = 0.0
    move.angular.x = 0.0
    move.angular.y = 0.0
    move.angular.z = radians(45)  # Positive for anticlockwise (left) rotation
    
    # Calculate the duration needed to turn 180 degrees at the given speed
    # Since we are turning at 45 degrees per second, we need 4 seconds to turn 180 degrees
    duration = rospy.Duration(4)  # Duration in seconds
    
    # Record the start time
    start_time = rospy.Time.now()
    
    # Continue turning until the desired duration has passed
    while rospy.Time.now() - start_time < duration:
        pub.publish(move)
        rate.sleep()
    



def obstacleavoid(x, y, z, move,pub):
    
    #if distance <= 0.75:
        
    if (abs(x)>0.5 and abs(y)>0.5):
        
        if (abs(x)<abs(y)): # closest wall infront or behind 
            print ("x is less than y") 
            if (x<0):
                print("turning 180 and moving forward")
                turn_180_degrees(move,pub) #rotate 180
                move_forward(x,y,z, move,pub)#go straight
            else:
                print("moving forward")
                move_forward(x,y,z, move,pub) #go straight
        else:
            print ("y is less than x")
            if (y<0):
                print ("turning right and moving forward")
                turn_90_degrees_clockwise(move,pub) #rotate 90 degrees clockwise
                move_forward(x,y,z, move,pub) #go straight
            else:
                print("turning left and moving forward")
                turn_90_degrees_anticlockwise(move,pub) #rotate 90 degrees anticlockwise
                move_forward(x,y,z,move,pub) #go straight

    move.linear.x =0
    move.angular.z=0
    pub.publish(move)

#    rospy.signal_shutdown("Node shutdown requested")



def checkDistance(data):
    move = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Initialize the publisher
    if (move.linear.x ==0 and move.angular.z ==0):
        x=0 
        y=0
        z=0

        # This function is triggered each time a new PointCloud2 message is received
        gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        min_distance = float('inf')
        
        for point in gen:
            x, y, z = point
           

       
            print (x,y,z)
        move_forward(x,y,z, move, pub)
        
        rospy.signal_shutdown("Node shutdown requested")

        #obstacleavoid(x, y,z, move,pub)
        #else:
         #   rospy.loginfo("No object detected directly in front.")
        del gen
        gc.collect()     


def main():

    rospy.init_node('robot_automove', anonymous=False)  # Initialize the node
    rospy.Subscriber('/velodyne_points', PointCloud2, checkDistance)  # Subscribe to the topic
    
    #rospy.Subscriber('/odom', Odometry, savingOdomData)  # Subscribe to the /odom topic
    #rospy.loginfo("Subscribed to /odom for pose information.")
    
    rate = rospy.Rate(50)

    print("Initialised Node Robot Automove ...")
    print("Subscribed to Velodyne Points")
    print("Entering while loop now")

    rospy.spin() # so that the code deesnt stop 
        

if __name__ == '__main__':
    main()


    #SUBSCRIBE TO pose
    #WHEN TURNING CHECK Z ROTATION DATA AND IDENTIFY THE GOAL ORIENTATION 
    #AND THEN LET IT ROTATE UNTIL IT REACHES THAT GOAL ORIENTATION WITH A FLAG 
