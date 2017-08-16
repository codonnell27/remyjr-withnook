#!/usr/bin/env python

import rospy
import time
import math
from mpu6050 import mpu6050
from std_msgs.msg import Bool
import numpy as np

imu = mpu6050(0x68)
jump_threshold = 3
accel_queue_len = 10

loop_period = 0.01
filter_scale = 0.98

def moveQueue():
	global accel_queue
	for i in range( len(accel_queue) -1):
		accel_queue[i] = accel_queue[i +1]
	accel_queue[len(accel_queue)-1] = 0

def findIfJumped(): 
	if (c_angle_z < c_angle_z - jump_threshold) | ( c_angle_z > ref_angle_z + jump_threshold):
		return True	
	else:
		return False

def calibrateGyro():
	global c_angle_x, c_angle_y, c_angle_z, ref_angle_x, ref_angle_y, ref_angle_z

	accel_data = imu.get_accel_data()
        c_angle_y = math.degrees(math.atan2(accel_data['x'], math.hypot(accel_data['y'], accel_data['z'])))
        c_angle_z = math.degrees(math.atan2(accel_data['y'], math.hypot(accel_data['z'], accel_data['x'])))
        c_angle_x = math.degrees( math.atan2(accel_data['z'], math.hypot(accel_data['x'], accel_data['y'])))
	
	ref_angle_x = c_angle_x
	ref_angle_z = c_angle_z
	ref_angle_y = c_angle_y

def getPosition():
	global c_angle_x, c_angle_y, c_angle_z
	accel_data = imu.get_accel_data()
	gyro_data = imu.get_gyro_data()
	g_angle_x_delta = gyro_data['x']*loop_period
	g_angle_y_delta = gyro_data['y']*loop_period
	g_angle_z_delta = gyro_data['z']*loop_period

	a_angle_y = math.degrees(math.atan2(accel_data['x'], math.hypot(accel_data['y'], accel_data['z'])))
	a_angle_z = math.degrees(math.atan2(accel_data['y'], math.hypot(accel_data['z'], accel_data['x'])))
	a_angle_x = math.degrees( math.atan2(accel_data['z'], math.hypot(accel_data['x'], accel_data['y'])))

	c_angle_x = filter_scale*(c_angle_x + g_angle_x_delta) + (1-filter_scale)*a_angle_x
        c_angle_y = filter_scale*(c_angle_y + g_angle_y_delta) + (1-filter_scale)*a_angle_y
        c_angle_z = filter_scale*(c_angle_z + g_angle_z_delta) + (1-filter_scale)*a_angle_z
	
	print c_angle_x, c_angle_y, c_angle_z
	
def start():
	global accel_queue
	calibrateGyro()
	accel_queue= np.zeros(accel_queue_len)
	pub = rospy.Publisher('remyjr/has_jumped', Bool, queue_size=1)
	rospy.init_node('gyroreader')
	rate = rospy.Rate(1/loop_period)	
	time.sleep(loop_period)
	i = 0
	while not rospy.is_shutdown():
		getPosition()
		has_jumped = Bool()
		jump_data = findIfJumped()		
		has_jumped.data = jump_data
		print has_jumped
	        pub.publish(has_jumped)
		rate.sleep()

if __name__ == '__main__':
	try:
		start()
	except KeyboardInterrupt:
		pass
