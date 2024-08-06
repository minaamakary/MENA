#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist

def explore_room():
    rospy.init_node('turtlebot3_explorer', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    move_cmd = Twist()

    rospy.loginfo("Starting exploration...")

    while not rospy.is_shutdown():
        # Set a random forward speed
        move_cmd.linear.x = random.uniform(0.2, 0.4)  # Move faster between 0.2 and 0.4 m/s
        move_cmd.angular.z = 0.0  # No rotation, move straight
        rospy.loginfo(f"Moving forward with speed {move_cmd.linear.x}")
        pub.publish(move_cmd)
        rospy.sleep(random.uniform(1, 3))  # Move forward for 1 to 3 seconds

        # Stop the robot briefly before turning
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.sleep(0.5)

        # Set a random angular speed for turning
        move_cmd.angular.z = random.choice([-1, 1]) * random.uniform(0.5, 1.0)  # Turn randomly left or right
        rospy.loginfo(f"Turning with angular speed {move_cmd.angular.z}")
        pub.publish(move_cmd)
        rospy.sleep(random.uniform(1, 2))  # Turn for 1 to 2 seconds

        # Stop the robot after turning
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        explore_room()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration terminated.")
