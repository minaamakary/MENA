#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class PoseOrientationNode:
    def __init__(self):
        # Initialize the ROS node (if not already initialized)
        if not rospy.core.is_initialized():
            rospy.init_node('pose_orientation_node', anonymous=True)
            rospy.loginfo("Robot Pose and Orientation Node Initialized.")

        # Publishers
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        rospy.loginfo("Initialized publisher for /robot_pose.")

        self.yaw_pub = rospy.Publisher('/robot_yaw', Float32, queue_size=10)
        rospy.loginfo("Initialized publisher for /robot_yaw.")

        # Subscriber
        # Note: If you want the subscriber to be active when the class is instantiated,
        # uncomment the following line:
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        rospy.loginfo("Received odometry data.")

        # Extract position
        position = data.pose.pose.position
        x = position.x
        y = position.y
        z = position.z

        # Extract orientation
        orientation = data.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]

        try:
            # Convert quaternion to Euler angles
            euler = tf.transformations.euler_from_quaternion(q)
            roll, pitch, yaw = euler
        except Exception as e:
            rospy.logerr(f"Error converting quaternion to Euler angles: {e}")
            return

        # Log the pose and orientation
        rospy.loginfo(f"Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        rospy.loginfo(f"Orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")

        # Publish pose as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = data.header
        pose_msg.pose = data.pose.pose
        self.pose_pub.publish(pose_msg)
        rospy.loginfo("Published PoseStamped message to /robot_pose.")

        # Publish yaw angle separately
        yaw_msg = Float32()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)
        rospy.loginfo("Published yaw angle to /robot_yaw.")

def main():
    node = PoseOrientationNode()

    # Subscribe to the odometry topic
    rospy.Subscriber('/odom', Odometry, node.odom_callback)
    rospy.loginfo("Subscribed to /odom topic.")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot Pose and Orientation Node terminated.")
