#!/usr/bin/env python

import rospy
import time
from remyJr.msg import tof_data
import RPi.GPIO as GPIO
#for some reason it was having trouble finding the VL53L0X package, hence these lines:
import sys
sys.path.append('/home/pi/github_packages/VL53L0X_rasp_python')
from python import VL53L0X

f_sensor_shdn = 12
b_sensor_shdn = 6

#these represent the distance that the floor distance
#any further and this node will report that the floor has diappeared from under the robot
f_floor_dist = 110 #in mm
b_floor_dist  =100 

f_tof = VL53L0X.VL53L0X(address=0x2B) #object number 0
b_tof = VL53L0X.VL53L0X(address=0x2D) #object number 1

def readTOF(): #reads time of flight sensors
	global f_distance, b_distance, b_drive_ok, f_drive_ok, last_b_ok, last_f_ok	
	f_distance = f_tof.get_distance()
	b_distance = b_tof.get_distance()

	# b_drive_ok and f_drive_ok represent whether it's okay to drive forward
	# there have to be two consecutive readings of a cliff before the node will report
	# that there is a cliff; this is to help prevent false positives
#	if b_distance > b_floor_dist:
#		if last_b_ok == False:
#			b_drive_ok = False
#		else: 
#			b_drive_ok = True
#		last_b_ok = False
#	else:
#		last_b_ok = True
#		b_drive_ok = True
#		
##	if f_distance > f_floor_dist:
###		if last_f_ok == False:
#			f_drive_ok = False
#		else: 
#			f_drive_ok = True
#		last_f_ok = False
#	
#	else:
#		last_f_ok = True
#		f_drive_ok = True
        if f_distance > f_floor_dist:
                f_drive_ok = False
        else:
                f_drive_ok = True


	if b_distance > b_floor_dist:
		b_drive_ok = False
	else:
		b_drive_ok = True

def destroy():
	f_tof.stop_ranging()
	b_tof.stop_ranging()
	
	GPIO.output(f_sensor_shdn, GPIO.LOW)
	GPIO.output(b_sensor_shdn, GPIO.LOW)
	GPIO.cleanup()

def start():
	global f_distance, b_distance, b_drive_ok, f_drive_ok
	f_distance = 0
	b_distance = 0
	b_drive_ok = False #reports False if it thinks the floor has diappeared from under the sensor
	f_drive_ok = False
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
	GPIO.setup(f_sensor_shdn, GPIO.OUT) 
	GPIO.setup(b_sensor_shdn, GPIO.OUT)
	GPIO.output(f_sensor_shdn, GPIO.LOW) #when the shutdown pins are low, the sensor doesn't range
	GPIO.output(b_sensor_shdn, GPIO.LOW)
	time.sleep(0.5)

	#starts the front sensor
        GPIO.output(f_sensor_shdn, GPIO.HIGH) 
        time.sleep(0.5)
        f_tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

	#starts the back sensor
	GPIO.output(b_sensor_shdn, GPIO.HIGH)
	time.sleep(0.5)
        b_tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

	pub = rospy.Publisher('remyjr/tof_data', tof_data, queue_size=1)
	rospy.init_node('tofreader')	
	rate = rospy.Rate(30)
	publish_count = 0
	while not rospy.is_shutdown():
	   try:
		readTOF()
		data = tof_data()
		data.publish_count = publish_count
		data.front_distance = f_distance
		data.back_distance = b_distance
		data.drive_back_ok = b_drive_ok
		data.drive_front_ok = f_drive_ok
		rospy.loginfo(data)
		pub.publish(data)
		publish_count +=1
		if publish_count >=2000:
			publish_count = 0
		rate.sleep()
	   except IOError, err:
		continue

if __name__ == '__main__':
	try:
		start()
	except KeyboardInterrupt:
		pass
