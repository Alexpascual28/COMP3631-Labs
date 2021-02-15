#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

def publisher():
	# Topics
	global velocity_topic
	velocity_topic = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
	# Node
	rospy.init_node('Walker', anonymous=True)
	# Variables
	rate = rospy.Rate(10) #10hz
	pi = 3.14159265359
	global desired_velocity
	desired_velocity = Twist()
	# Loop
	while not rospy.is_shutdown():
		rospy.Subscriber('mobile_base/events/bumper', BumperEvent, processBump)
		desired_velocity.linear.x = 0.3
		desired_velocity.angular.z = 0
		for i in range(40):
			velocity_topic.publish(desired_velocity)
			rate.sleep()
		desired_velocity.linear.x = 0
		desired_velocity.angular.z = 90 * pi/180
		for i in range(10):
			velocity_topic.publish(desired_velocity)
			rate.sleep()

def processBump(data):
	if (data.state == BumperEvent.PRESSED):
		rospy.loginfo(data.state)
		while True:
			desired_velocity.linear.x = 0
			desired_velocity.angular.z = 0
			velocity_topic.publish(desired_velocity)

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
