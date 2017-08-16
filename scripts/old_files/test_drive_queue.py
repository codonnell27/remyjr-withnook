#!/usr/bin/env python
from __future__ import division
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import rospy
import numpy as np
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

TRIG = 18
ECHO = 17

drivePin = 0
turnPin = 1

speed_queue_length = 10

drive_speed_max = 330
drive_speed_min = 300
drive_speed_zero = 315
drive_speed_zero_uplim = 320
drive_speed_zero_lowlim = 310


tooCloseDis = 45

pwm = Adafruit_PCA9685.PCA9685() 

def setup():
	global speed_queue, joy_a, joy_l, iteration_num, lastDis
	speed_queue = np.zeros((speed_queue_length,))
	joy_a = 0
	joy_l = 0
	lastDis = 0
	iteration_num = 0
	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        pwm.set_pwm_freq(50)

def queueAvg():
	global speed_queue
	sum = 0
	for i in range(0, len(speed_queue)-1):
		speed_queue[i] = speed_queue[i+1]
		sum = sum + speed_queue[i]
	speed_queue[len(speed_queue)-1] = joy_l
	sum = sum + joy_l
	average = sum / len(speed_queue)
	return average

def decelerate():
	global joy_l, iteration_num 
	joy_l = 0
	pwm.set_pwm(turnPin, 0, drive_speed_zero)
	new_speed = queueAvg()
	if new_speed > 0:
		joy_l = -1
	elif new_speed < 0:
		joy_l = 1
	
	for i in range(speed_queue_size):
		new_v = calcPWM(new_speed)
		iteration_num = iteration_num+1
                print ('%d : Drive speed: %d Turn speed: %d') %(iteration_num, new_v, drive_speed_zero)
		pwm.set_pwm(drivePin, 0, new_v)
		new_speed = queueAvg()

	joy_l = 0
		
def calcPWM(joy_value):
	if joy_value > 0:
	        pwm_val = int (joy_value *(drive_speed_max - drive_speed_zero_uplim) + drive_speed_zero_uplim)

	elif joy_value < 0:
		pwm_val = int (-1*joy_value *(drive_speed_zero_lowlim - drive_speed_min) + drive_speed_min)
	else:
		pwm_val = 0
	return pwm_val

def motion():
	print ('Driving')
	global speed_queue, joy_l, iteration_num
	joy_l_avg = queueAvg()
	print ('Turn: %d, drive: %d, drive avg: %d')%(joy_a, joy_l, joy_l_avg)
	
	if  (joy_l_avg !=0) & (okToDrive()):#there is a command to drive and no obstacles
		v_l = calcPWM(joy_l_avg)
                print ('%d: Drive speed: %d Turn speed: %d') %(iteration_num, v_l, drive_speed_zero)
		pwm.set_pwm(turnPin, 0, drive_speed_zero)
		pwm.set_pwm(drivePin, 0, v_l)	
		iteration_num = iteration_num+1


	elif (joy_a !=0):
		if  (joy_l_avg !=0):
			decelerate()
		v_a = calcPWM(joy_a)
                print ('%d: Drive speed: %d Turn speed: %d') %(iteration_num, drive_speed_zero, v_a)
		pwm.set_pwm(drivePin, 0, drive_speed_zero)
		pwm.set_pwm(turnPin, 0, v_a)
		iteration_num = iteration_num + 1


	else:
		if  (joy_l_avg !=0):
			decelerate()
                print ('%d: Drive speed: %d Turn speed: %d') %(iteration_num, drive_speed_zero, drive_speed_zero)
		pwm.set_pwm(drivePin, 0, drive_speed_zero)
		pwm.set_pwm(turnPin, 0, drive_speed_zero)
		iteration_num = iteration_num + 1
		


def callback(data):
	global pub, joy_l, joy_a
	#twist = Twist()
	joy_l = data.linear.x
	joy_a = data.angular.z 
	#pub.publish(twist)

def okToDrive():
	global lastDis
        GPIO.output(TRIG, 0)
        time.sleep(0.000002)
        GPIO.output(TRIG, 1)
        time.sleep(0.00001)
        GPIO.output(TRIG, 0)

        while GPIO.input(ECHO) == 0:
                a = 0
        time1 = time.time()
        while GPIO.input(ECHO) == 1:
                a = 1
        time2 = time.time()
        during = time2 - time1
        dis =  during *340 /2 *100
	if (dis > 1000) | (dis < 10):
		print ( '%d, !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!') % (dis)
	else:
		print dis
	notTooClose = False
	if dis > tooCloseDis:
		notTooClose = True
	elif lastDis > tooCloseDis:
		notTooClose = True
	lastDis = dis
	return notTooClose


def listener():
	print ('Listening...')
	global pub
	#pub = rospy.Publisher('remyjr/pwm_value', Twist)
	rospy.init_node('cmd_vel_listener')
	rospy.Subscriber("remyjr/cmd_vel", Twist, callback)


def mainLoop():
	while not rospy.is_shutdown():
		listener()
		motion()
		time.sleep(0.1)
	destroy()

def destroy():
        GPIO.cleanup()

if __name__ == "__main__":
	print('Setting up...')
        setup()
	print ('Setup Complete!')
        try:
		mainLoop()        
        except KeyboardInterrupt:
		destroy()
		pass



