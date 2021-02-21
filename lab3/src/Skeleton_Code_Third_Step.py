#!/usr/bin/env python
# This third piece of skeleton code will be centred around the properties of the object, and determining
# when a colour object is detected, to set a flag. 

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():
    def __init__(self):
        # Initialise a publisher to publish messages or print the message when the colour is detected
        # We covered which topic receives messages in the 1st Lab Session
		self.pub = rospy.Publisher('chatter', String, queue_size=10)

        # Initialise any flags that signal a colour has been detected in view (default to false)
		self.red_detected = False
		self.green_detected = False
		self.blue_detected = False

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

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
		red_contours = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		green_contours = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		blue_contours = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		# total_contours = cv2.findContours(mask_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		
        # Loop over the contours
		# Green
		if len(green_contours[0]) > 0:
            # Different methods for identifying biggest contour:
            # Loop through list and keep track of biggest contour
            #OR
            # Use max() method to find largest contour
			max_contour = max(green_contours[0], key=cv2.contourArea)
            #Moments can calculate the center of the contour
			M = cv2.moments(max_contour)
			center_x, center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
			if cv2.contourArea(max_contour) > 100: #<What do you think is a suitable area?>:
                # draw a circle on the contour you're identifying as a green object as well
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
				(x, y), radius = cv2.minEnclosingCircle(max_contour)
				cv2.circle(mask_image, (center_x,center_y), int(radius), (255, 255, 0), 1)
                # Then alter the values of any flags
				self.green_detected = True
		else:
			self.green_detected = False

		# Blue
		if len(blue_contours[0]) > 0:
			max_contour = max(blue_contours[0], key=cv2.contourArea)
			M = cv2.moments(max_contour)
			center_x, center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			if cv2.contourArea(max_contour) > 100:
				(x, y), radius = cv2.minEnclosingCircle(max_contour)
				cv2.circle(mask_image, (center_x,center_y), int(radius), (255, 255, 0), 1)
				self.blue_detected = True
		else:
			self.blue_detected = False
			
		# Red
		if len(red_contours[0]) > 0:
			max_contour = max(red_contours[0], key=cv2.contourArea)
			M = cv2.moments(max_contour)
			center_x, center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			if cv2.contourArea(max_contour) > 100:
				(x, y), radius = cv2.minEnclosingCircle(max_contour)
				cv2.circle(mask_image, (center_x,center_y), int(radius), (255, 255, 0), 1)
				self.red_detected = True
		else:
			self.red_detected = False
		
		# Print state of flags
		# Green
        #if the flag is true (colour detected)
		if self.green_detected == True:
			#print the flag to check it works
			rospy.loginfo("Green Flag: %s", self.green_detected)
			#alternatively publish to the lab1 talker/listener
			self.pub.publish("Green Flag: %s" % self.green_detected)
			
		# Blue
		if self.blue_detected == True:
			rospy.loginfo("Blue Flag: %s", self.blue_detected)
			self.pub.publish("Blue Flag: %s" % self.blue_detected)
			
		# Red
		if self.red_detected == True:
			rospy.loginfo("Red Flag: %s", self.red_detected)
			self.pub.publish("Red Flag: %s" % self.red_detected)

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
	rospy.init_node('third_step', anonymous=True)
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
