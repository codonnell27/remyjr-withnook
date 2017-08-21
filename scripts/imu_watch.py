#!/usr/bin/env python
import logging
import sys
import time
import rospy
from remyJr.msg import imu_data
from Adafruit_BNO055 import BNO055
import math


# system dependent variables:

#the acceleration due to gravity in the x-direction when the robot is at rest
# if the acceleration due to gravity in the x-direction is more than this value away from the level
# value then this node reports that the robot is no longer level
# If this code is used on a different system then this may need to be changed.
level_grav_x = -0.5

# This determines how far away from level_grav_x the robot needs to be before it is no longer
# considered level. If this code is used on a different robot then the original remyJr then
# this number may need to change depending on how close to the axis of rotation the imu is
jump_threshold = 1.5

reset_pin = 21

# System independent Variables

loop_freq = 20 # in hz

def findIfLevel():
	#decides whether the robot is level
	if (grav_x < level_grav_x-jump_threshold) | (grav_x > level_grav_x + jump_threshold):
		return False
	else:
		return True

def start():
	global grav_x

	# Create and configure the BNO sensor connection.
	# Raspberry Pi configuration with serial UART and RST connected to GPIO 21:
	bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=reset_pin)


	# Initialize the BNO055 and stop if something went wrong.
	if not bno.begin():
	    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

	# Print system status and self test result.
	status, self_test, error = bno.get_system_status()
	print('System status: {0}'.format(status))
	print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
	# Print out an error if system status is in error mode.
	if status == 0x01:
	    print('System error: {0}'.format(error))
	    print('See datasheet section 4.3.59 for the meaning.')

	# Print BNO055 software revision and other diagnostic data.
	sw, bl, accel, mag, gyro = bno.get_revision()
	print('Software version:   {0}'.format(sw))
	print('Bootloader version: {0}'.format(bl))
	print('Accelerometer ID:   0x{0:02X}'.format(accel))
	print('Magnetometer ID:    0x{0:02X}'.format(mag))
	print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


	pub = rospy.Publisher('remyjr/imu_data', imu_data, queue_size = 1)
	rospy.init_node('imureader')
	rate = rospy.Rate(loop_freq)
	publish_count = 0

	print('Reading BNO055 data, press Ctrl-C to quit...')
	while not rospy.is_shutdown():
		# Read the Euler angles for heading, roll, pitch (all in degrees).
		heading, roll, pitch = bno.read_euler()
		# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
		sys, gyro, accel, mag = bno.get_calibration_status()
		#gets the acceleration due to gravity in each direction
		grav_x, grav_y, grav_z = bno.read_gravity()
		# an exaustive list of possible bno functions can be found in the example
		# in the bno package

		data = imu_data()
		data.publish_count = publish_count
                data.is_level = findIfLevel()
		rospy.loginfo(data)
                pub.publish(data)
                publish_count += 1

		# Sleep for a bit until the next reading.
		rate.sleep()

if __name__ == '__main__':
	try:
		start()
	except KeyboardInterrupt:
		pass
