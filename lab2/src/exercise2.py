#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	pi = 3.14159265359
	desired_velocity = Twist()
	while not rospy.is_shutdown():
		desired_velocity.linear.x = 0.3
		desired_velocity.angular.z = 0
		for i in range(40):
			pub.publish(desired_velocity)
			rate.sleep()
		desired_velocity.linear.x = 0
		desired_velocity.angular.z = 90 * pi/180
		for i in range(10):
			pub.publish(desired_velocity)
			rate.sleep()

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
