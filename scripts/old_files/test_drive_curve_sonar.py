#!/usr/bin/env python
from __future__ import division
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from remyJr.msg import sonar_array
import rospy
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685


num_of_sonar = 8

#for the motor controllers
drivePin = 0
turnPin = 1

#PWM values
drive_speed_max = 330 
drive_speed_min = 300
drive_speed_zero = 315 
drive_speed_zero_uplim = 320 #upper bound of the zero range
drive_speed_zero_lowlim = 310 #lower bound of the zero range

accel_const = 0.1 #determines the slope of the acceleration curve, must be between 0 and 1
tooCloseDis = 45 #in cm :when the sonars are closer than this, the robot won't drive straight. Must be greater then 30 for accurate results
sleep_time = 0.05 #the time the motors rest between changing PWM values

pwm = Adafruit_PCA9685.PCA9685() 

def setup():
	global joy_a, joy_l, motor_change_num, current_l, sonar_dis, obstacles
        global f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar
        f_sonar = 0
        f_r_sonar = 0
        r_sonar = 0
        b_r_sonar = 0
        b_sonar = 0
        b_l_sonar = 0
        l_sonar = 0
        f_l_sonar = 0
	joy_a = 0 #the last turning value from the joystick (joystick signal, so between -1 and 1)
	joy_l = 0 #the last drive value from the joystick
	current_l = 0 #the drive value that the motor is using (like the joystick signals, it's between -1 and 1)
	sonar_dis = [0]*num_of_sonar
	obstacles = [True]*num_of_sonar #holds which sonars detect obstacles. True means obstacle detected
	motor_change_num = 0 #coutner for how many times the motor has changed speeds
	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
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
	print obstacles
	print sonar_dis
	if (joy_l>0) & (joy_a == 0) & (not obstacles[f_sonar]):#there is a command to drive and no obstacles
		accelerate(joy_l)
	elif (joy_l < 0) & (joy_a == 0) &(not obstacles[b_sonar]):
		accelerate(joy_l) 
        elif (joy_l>0) & (joy_a > 0) & (not obstacles[f_r_sonar]):#there is a command to drive and no obstacles
                accelerate(joy_l)
        elif (joy_l < 0) & (joy_a > 0) & (not obstacles[b_r_sonar]):
                accelerate(joy_l)
        elif (joy_l>0) & (joy_a < 0) & (not obstacles[f_l_sonar]):#there is a command to drive and no obstacles
                accelerate(joy_l)
        elif (joy_l < 0) & (joy_a < 0) & (not obstacles[b_l_sonar]):
                accelerate(joy_l)
	else:
		accelerate(0)

def callback(data):
	global joy_l, joy_a
	joy_l = data.linear.x
	joy_a = data.angular.z 

def sonarlistener(data):
	global f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar, sonar_dis, obstacles
	f_sonar = data.pins[0]
	f_r_sonar = data.pins[1]
	r_sonar = data.pins[2]
	b_r_sonar = data.pins[3]
	b_sonar = data.pins[4]
	b_l_sonar = data.pins[5]
	l_sonar = data.pins[6]
	f_l_sonar = data.pins[7]

	sonar_dis = data.distances

	for i in range(len(sonar_dis)):
		if sonar_dis[i] > tooCloseDis:
			obstacles[i] = False
		else:
			obstacles[i] = True
	
def listener():
	print ('Listening...')
	rospy.init_node('cmd_vel_listener')
	rospy.Subscriber("remyjr/cmd_vel", Twist, callback, queue_size = 1)
	rospy.Subscriber("remyjr/sonar_dist", sonar_array, sonarlistener, queue_size = 1)

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



