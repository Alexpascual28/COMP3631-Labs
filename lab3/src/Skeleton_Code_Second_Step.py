#!/usr/bin/env python
# This second piece of skeleton code will be centred around
# combining colours and creating a mask

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():
    def __init__(self):
        # Initialise self.sensitivity colour detection value (10 should be enough)
		self.sensitivity = 10
        # Initialise a CvBridge()
		self.bridge = CvBridge()
        # Set up a subscriber to the image topic you wish to use
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

    def callback(self, data):
        # Convert received image into opencv image
        # Wrap call to conversion method in exception handler
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as error:
			print(error)
			
        # Set upper and lower bounds for green colour (HSV)
		hsv_green_lower = np.array([60-self.sensitivity, 100, 100])
		hsv_green_upper = np.array([60+self.sensitivity, 255, 255])
        # Set the upper and lower bounds for red and blue
        ##Note green = 60, blue = 120, red = 0 for HSV in OpenCV
		hsv_blue_lower = np.array([120-self.sensitivity, 100, 100])
		hsv_blue_upper = np.array([120+self.sensitivity, 255, 255])
		hsv_red_lower = np.array([0-self.sensitivity, 100, 100])
		hsv_red_upper = np.array([0+self.sensitivity, 255, 255])
		
        # Convert rgb image into hsv image
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Filter out colours separately using cv2.inRange() method
		green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
		blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
		red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        # Combine masks using cv2.bitwise_or() method (TWO images at once)
		gb_mask = cv2.bitwise_or(green_mask, blue_mask)
		rgb_mask = cv2.bitwise_or(red_mask, gb_mask)
        # Apply mask to original image using cv2.bitwise_and() method
        # bitwise_and image with itself and pass the mask to the mask parameter
        # instead of performing a bitwise_and on the mask and the image.
		mask_image=cv2.bitwise_and(cv_image,cv_image, mask=rgb_mask)

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow('camera_Feed')
		cv2.imshow('camera_Feed', mask_image)
		cv2.waitKey(3)
		
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate class
	cI = colourIdentifier()
    # And rospy.init the entire node
	rospy.init_node('second_step', anonymous=True)
    # Ensure that node continues running with rospy.spin()
    # Wrap rospy.spin() in exception handler for KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Closing Program")
    # Destroy image windows before closing node
	cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
