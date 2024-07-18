#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import cv2

def resize_image(image_path, width, height, output_path):
    image = cv2.imread(image_path)
    if image is None:
        rospy.logerr(f"Failed to read image from {image_path}")
        return

    # Convert wall dimensions from meters to pixels (if needed)
    # Assuming 1 meter = 100 pixels for this example
    width_px = int(width * 100)
    height_px = int(height * 100)

    # Resize the image to fit the wall dimensions
    resized_image = cv2.resize(image, (width_px, height_px))

    # Save the resized image to the specified output path
    cv2.imwrite(output_path, resized_image)
    rospy.loginfo(f"Resized image saved to {output_path}")
    cv2.imshow(output_path, image)


def wall_dimensions_callback(data):
    width, height = data.data

    # Define the path to your image and the output path
    image_path = "/home/minamakary/ros/catkin_ws/src/thesis/images/portrait_one.jpg"  # Update this path to your image file
    output_path = "resized_wall_image.jpg"

    # Resize the image based on the received wall dimensions
    resize_image(image_path, width, height, output_path)

def main():
    rospy.init_node('image_resizer_node', anonymous=True)
    rospy.Subscriber('/wall_dimensions', Float32MultiArray, wall_dimensions_callback)
    rospy.loginfo("Image Resizer node initialized and subscribed to /wall_dimensions")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
