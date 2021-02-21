#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.

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
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 2nd Lab Session
		self.velocity = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        # Initialise any flags that signal a colour has been detected in view (default to false)
		self.red_detected = False
		self.green_detected = False
		self.blue_detected = False
		
		# Initiate global variables
		self.desired_velocity = Twist()

        # Initialise standard movement messages such as a simple move forward and a message with all zeroes (stop)
		self.forward = 0.3
		self.backward = -0.3
		self.left = 0.4
		self.right = -0.4
		self.stop = 0

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

        # Loop over the contours
		# Green
		if len(green_contours[0]) > 0:
            # Use max() method to find largest contour
			max_green_contour = max(green_contours[0], key=cv2.contourArea)
            #Moments can calculate the center of the contour
			M = cv2.moments(max_green_contour)
			green_center_x, green_center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
			if cv2.contourArea(max_green_contour) > 100:
                # draw a circle on the contour you're identifying
				(x, y), radius = cv2.minEnclosingCircle(max_green_contour)
				cv2.circle(mask_image, (green_center_x, green_center_y), int(radius), (255, 255, 0), 1)
                # Then alter the values of any flags
				self.green_detected = True
		else:
			self.green_detected = False
			
		# Blue
		if len(blue_contours[0]) > 0:
			max_blue_contour = max(blue_contours[0], key=cv2.contourArea)
			M = cv2.moments(max_blue_contour)
			blue_center_x, blue_center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			if cv2.contourArea(max_blue_contour) > 100:
				(x, y), radius = cv2.minEnclosingCircle(max_blue_contour)
				cv2.circle(mask_image, (blue_center_x, blue_center_y), int(radius), (255, 255, 0), 1)
				self.blue_detected = True
		else:
			self.blue_detected = False
			
		# Red
		if len(red_contours[0]) > 0:
			max_red_contour = max(red_contours[0], key=cv2.contourArea)
			M = cv2.moments(max_red_contour)
			red_center_x, red_center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			if cv2.contourArea(max_red_contour) > 100:
				(x, y), radius = cv2.minEnclosingCircle(max_red_contour)
				cv2.circle(mask_image, (red_center_x,red_center_y), int(radius), (255, 255, 0), 1)
				self.red_detected = True
		else:
			self.red_detected = False

        #If red flag is true, stop the robot
		if self.red_detected == False:
			# If green flag is true, move towards green ball
			if self.green_detected == True:
				if cv2.contourArea(max_green_contour) > 145000:
					# Too close to object, need to move backwards
					self.desired_velocity.linear.x = self.backward
				elif cv2.contourArea(max_green_contour) < 140000:
					# Too far away from object, need to move forwards
					self.desired_velocity.linear.x = self.forward
				else:
					self.desired_velocity.linear.x = self.stop
					
				#Alternatively cx,cy can be used to position and rotate towards the object
				if green_center_x < 320:
					self.desired_velocity.angular.z = self.left
				elif green_center_x > 320:
					self.desired_velocity.angular.z = self.right
				else:
					self.desired_velocity.angular.z = self.stop
					
			# If green flag is false but blue fag is true, move towards blue ball
			elif self.blue_detected == True:
				if cv2.contourArea(max_blue_contour) > 145000:
					# Too close to object, need to move backwards
					self.desired_velocity.linear.x = self.backward
				elif cv2.contourArea(max_blue_contour) < 140000:
					# Too far away from object, need to move forwards
					self.desired_velocity.linear.x = self.forward
				else:
					self.desired_velocity.linear.x = self.stop
					
				#Alternatively cx,cy can be used to position and rotate towards the object
				if blue_center_x < 320:
					self.desired_velocity.angular.z = self.left
				elif blue_center_x > 320:
					self.desired_velocity.angular.z = self.right
				else:
					self.desired_velocity.angular.z = self.stop
					
			# If nothing is detected, stop
			else:
				self.desired_velocity.linear.x = self.stop
				self.desired_velocity.angular.z = self.stop
		else:
			self.desired_velocity.linear.x = self.stop
			self.desired_velocity.angular.z = self.stop
				
		self.velocity.publish(self.desired_velocity)
		
		#rospy.loginfo("Green_Center_X: %s" % green_center_x)
		#rospy.loginfo("Green_Contour: %s" % cv2.contourArea(max_green_contour))
		# if self.red_detected == True:
			# rospy.loginfo("Red_Center_X: %s" % red_center_x)
            
        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow('camera_Feed')
		cv2.imshow('camera_Feed', mask_image)
		cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # And rospy.init the entire node
	rospy.init_node('final_step', anonymous=True)
    # Instantiate class
	cI = colourIdentifier()
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
