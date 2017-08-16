#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

joystick_linear_axis = 1
joystick_angular_axis = 2




def callback(data):
	twist = Twist()
	twist.linear.x = data.axes[joystick_linear_axis]
	twist.angular.z = data.axes[joystick_angular_axis]
	pub.publish(twist)

def start():
	global pub
	pub = rospy.Publisher('remyjr/cmd_vel', Twist, queue_size=5)
	rospy.init_node('Joy2Remy')
	rospy.Subscriber ("joy", Joy, callback)		
	rospy.spin()

if __name__ == '__main__':
	try:
		start()
	except rospy.ROSInterruptException:
		pass
