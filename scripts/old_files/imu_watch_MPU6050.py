#!/usr/bin/env python
import rospy
import time
import math
from mpu6050 import mpu6050
from remyJr.msg import imu_data
import numpy as np


imu = mpu6050(0x68)
jump_threshold = 3 #the range from baseline that the robot has to move before this program will no longer consider it level
turn_threshold = 2
loop_period = 0.01 #how quickly it cycles through the program 

# this value determines how much more the gyro data is trusted over the accelerometer 
# over a the short term when calculating the position
# a higher value (must be between 1 and 0) means that the gyro data is trusted more in the short term
filter_scale = 0.98 

def findIfJumped(): #returns false if the robot isn't level 
	if (c_angle_y < ref_angle_y - jump_threshold) | ( c_angle_y > ref_angle_y + jump_threshold):
		return True	
	else:
		return False

def calibrateGyro(): #sets the "level" values, so the robot must be level when this runs (during setup)
	global c_angle_x, c_angle_y, c_angle_z, ref_angle_x, ref_angle_y, ref_angle_z, gyro_x_ref
	gyro_data = imu.get_gyro_data()
	accel_data = imu.get_accel_data()
        c_angle_y = math.degrees(math.atan2(accel_data['x'], math.hypot(accel_data['y'], accel_data['z'])))
        c_angle_z = math.degrees(math.atan2(accel_data['y'], math.hypot(accel_data['z'], accel_data['x'])))
        c_angle_x = math.degrees( math.atan2(accel_data['z'], math.hypot(accel_data['x'], accel_data['y'])))
	
	gyro_x_ref = gyro_data['x']	

	ref_angle_x = c_angle_x
	ref_angle_z = c_angle_z
	ref_angle_y = c_angle_y

def getPosition(): #calculates the orientation of the imu
	global c_angle_x, c_angle_y, c_angle_z, accel_data, g_angle_x_delta, g_angle_y_delta, g_angle_z_delta
	accel_data = imu.get_accel_data()
	gyro_data = imu.get_gyro_data()
	g_angle_x_delta = gyro_data['x']*loop_period
	g_angle_y_delta = gyro_data['y']*loop_period
	g_angle_z_delta = gyro_data['z']*loop_period

	a_angle_y = math.degrees(math.atan2(accel_data['x'], math.hypot(accel_data['y'], accel_data['z'])))
	a_angle_z = math.degrees(math.atan2(accel_data['y'], math.hypot(accel_data['z'], accel_data['x'])))
	a_angle_x = math.degrees( math.atan2(accel_data['z'], math.hypot(accel_data['x'], accel_data['y'])))
	
	#filters the two values to trust the gyro more in the short term
	c_angle_x = filter_scale*(c_angle_x + g_angle_x_delta) + (1-filter_scale)*a_angle_x
        c_angle_y = filter_scale*(c_angle_y + g_angle_y_delta) + (1-filter_scale)*a_angle_y
        c_angle_z = filter_scale*(c_angle_z + g_angle_z_delta) + (1-filter_scale)*a_angle_z
	
#	print c_angle_x, c_angle_y, c_angle_z
	
def start():
	calibrateGyro()
	time.sleep(0.1) #when the accelerometer first turns on it can take a while for the values to settle
	calibrateGyro()
	pub = rospy.Publisher('remyjr/imu_data', imu_data, queue_size=1)
	rospy.init_node('imureader')
	rate = rospy.Rate(1/loop_period)	
	time.sleep(loop_period)
	publish_count = 0 #this is the 'heartbeat' to check if the sensor is online
	print_count = 0
	while not rospy.is_shutdown():
	    try:
		getPosition()
		data = imu_data()
		data.publish_count = publish_count
		data.has_jumped = findIfJumped()		
		data.x_orientation = c_angle_x
		data.y_orientation = c_angle_y
		data.z_orientation = c_angle_z
		data.z_gyro = g_angle_z_delta
                data.y_gyro = g_angle_y_delta
		if (g_angle_x_delta < (gyro_x_ref - turn_threshold)) | (g_angle_x_delta > (gyro_x_ref + turn_threshold)):
                	data.x_gyro = g_angle_x_delta
	        else:
			data.x_gyro = 0
		pub.publish(data)
		publish_count += 1
		if publish_count >= 20000:
			publish_count = 0
		if print_count >=35: #prints the data at a readable rate
			print accel_data
			print "current angles" ,c_angle_x, c_angle_y, c_angle_z
			print "ref angles: ", ref_angle_x, ref_angle_y, ref_angle_z
			print_count = 0
		print_count += 1
		rate.sleep()
	    except IOError, err:
		continue


if __name__ == '__main__':
	try:
		start()
	except KeyboardInterrupt:
		pass
