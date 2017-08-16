#!/usr/bin/env python
from __future__ import division
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import rospy
import numpy as np
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

#for the sonar
TRIG = 18
ECHO = 17

#for the motor controllers
drivePin = 0
turnPin = 1

#PWM values
drive_speed_max = 330 
drive_speed_min = 300
drive_speed_zero = 315 
drive_speed_zero_uplim = 320 #upper bound of the zero range
drive_speed_zero_lowlim = 310 #lower bound of the zero range

accel_const = 0.9 #determines the slope of the acceleration curve, must be between 0 and 1
tooCloseDis = 45 #in cm :when the sonars are closer than this, the robot won't drive straight
sleep_time = 0.02 #the time the motors rest between changing PWM values

pwm = Adafruit_PCA9685.PCA9685() 

def setup():
	global joy_a, joy_l, motor_change_num, lastDis, current_l
	joy_a = 0 #the last turning value from the joystick (joystick signal, so between -1 and 1)
	joy_l = 0 #the last drive value from the joystick
	current_l = 0 #the drive value that the motor is using (like the joystick signals, it's between -1 and 1)
	lastDis = 0 #the last distance the sonars read, used to control for signal noise
	motor_change_num = 0 #coutner for how many times the motor has changed speeds
	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        pwm.set_pwm_freq(50)
	setMotors(0,0)

def accelerate(target_joy): #changes the speed of the motor with a fixed max rate (determined by accel_const)
	global current_l 
	if target_joy > current_l:
		while current_l < target_joy:
              		setMotors(current_l, joy_a)
			current_l = current_l + accel_const
	if target_joy < current_l: 
                while current_l> target_joy:
			setMotors(current_l, joy_a)
                        current_l = current_l - accel_const
	current_l = target_joy
	setMotors(current_l, joy_a)


def setMotors(joy_drive, joy_turn): #sets the motors to a new PWM and prints some info 
	global motor_change_num
	new_v_drive = calcPWM(joy_drive)
	new_v_turn = calcPWM(joy_turn)
        motor_change_num = motor_change_num+1
        print ('Turn signal: %f, drive signal: %f, drive speed: %f'%(joy_a, joy_l, current_l)) #prints (in order) the signal from the joystick for turning, the signal for driving, the value currently being used to determine drive speed
        print ('%d : Drive speed: %d Turn speed: %d' %(motor_change_num, new_v_drive, new_v_turn)) #prints (in order) the motor change number, drive PWM value, turn PWM value
        pwm.set_pwm(drivePin, 0, new_v_drive)
	pwm.set_pwm(turnPin, 0, new_v_turn)
	time.sleep(sleep_time) #sleeps for a set amount of time

def calcPWM(joy_value): #calculates the PWM value for a joystick value, skips the range of zero PWM values unless joy_value is zero
        if joy_value > 0:
                pwm_val = int (joy_value *(drive_speed_max - drive_speed_zero_uplim) + drive_speed_zero_uplim)
        elif joy_value < 0:
                pwm_val = int (joy_value *(drive_speed_zero_lowlim - drive_speed_min) + drive_speed_zero_lowlim)
        else:
                pwm_val = drive_speed_zero
        return pwm_val

def motion(): #main driving function 
	print ('Driving')
	
	if (joy_l!=0) & (okToDrive()):#there is a command to drive and no obstacles
		accelerate(joy_l)

	else:
		accelerate(0)

def callback(data):
	global joy_l, joy_a
	joy_l = data.linear.x
	joy_a = data.angular.z 

def okToDrive(): #reads sonar and determines if it is too close to something to drive
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
	if (dis < tooCloseDis): 
		print ( '%d, !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!') % (dis)
	else:
		print dis
	notTooClose = False
	if dis > tooCloseDis:
		notTooClose = True
	elif lastDis > tooCloseDis:
		notTooClose = True
		#sometimes when there is nothing less then a meter away,
		# the sonar will return a really high distance (>1000cm), then a 
		#really low distance (< 10cm) in succession, resulting in a false
		# reading of a close object. This controls against that 
	lastDis = dis
	return notTooClose


def listener():
	print ('Listening...')
	rospy.init_node('cmd_vel_listener')
	rospy.Subscriber("remyjr/cmd_vel", Twist, callback)


def mainLoop(): 
	while not rospy.is_shutdown():
		listener()
		motion()
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



