#!/usr/bin/env python
import logging
import sys
import time
import rospy
from remyJr.msg import imu_data
from Adafruit_BNO055 import BNO055
import math
level_grav_x = -0.51 #the acceleration due to gravity in the x-direction when the robot is at rest  
jump_threshold = 1.5
turn_threshold = 2
loop_period = 0.05



def findIfLevel():
	#decides whether the robot is level
	if (grav_x < level_grav_x-jump_threshold) | (grav_x > level_grav_x + jump_threshold):
		return False
	else:	
		return True

def findIfTurning():
	#decides whether the robot is turning
	global last_heading, heading_delta
	isTurning = False	
	heading_delta = heading - last_heading
	#this next part prevents giant heading_delta that results from the robot 
	#going from a ~350 degree heading to a ~10 degree heading or vice versa
	if heading_delta > 180:
		heading_delta = heading_delta -360
	elif heading_delta < -180:
		heading_delta = 360 + heading_delta 
	if (heading_delta < -turn_threshold) | (heading_delta > turn_threshold):
		isTurning = True
	last_heading = heading
	return isTurning

def start():
	global heading, pitch, roll, last_heading, heading_delta, grav_x
	heading = 1000
	pitch = 1000
	roll = 1000
	last_heading = 1000
	heading_delta = 1000
	
	# Create and configure the BNO sensor connection.  
	# Raspberry Pi configuration with serial UART and RST connected to GPIO 21:
	bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=21)


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
	rate = rospy.Rate(1/loop_period)
	publish_count = 0

	print('Reading BNO055 data, press Ctrl-C to quit...')
	while not rospy.is_shutdown():
		# Read the Euler angles for heading, roll, pitch (all in degrees).
		heading, roll, pitch = bno.read_euler()
		# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
		sys, gyro, accel, mag = bno.get_calibration_status()
		#gets the acceleration due to gravity in each direction
		grav_x, grav_y, grav_z = bno.read_gravity()		
		mag_x, mag_y, mag_z = bno.read_magnetometer()
		
		data = imu_data()
		data.publish_count = publish_count
                data.is_level = findIfLevel()
		rospy.loginfo(data)
                pub.publish(data)
                publish_count += 1

		print mag_x, mag_y, mag_z
		# Sleep for a second until the next reading.
		rate.sleep()

if __name__ == '__main__':
	try: 
		start()
	except KeyboardInterrupt:
		pass
