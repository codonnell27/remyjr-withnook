#!/usr/bin/env python
from __future__ import division
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from remyJr.msg import sonar_array
import rospy
import RPi.GPIO as GPIO
import time
import math
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
edgefind_speed = 0.5 #how fast the robot drives while edgefinding

accel_const = 0.1 #determines the slope of the acceleration curve, must be between 0 and 1
tooCloseDis = 50 #in cm :when the sonars are closer than this, the robot won't drive straight. Must be greater then 30 for accurate results
sleep_time = 0.05 #the time the motors rest between changing PWM values
sleep_time_edgefinding = 0.01 #the time motos rest between changing PWM values when edgefinding
right_angle = 90
#these next two values control what the robot consideres "right" for edgefinding
# 112 & 68 make a space of ~45 degrees
right_side_up_lim = 130 
right_side_low_lim = 55
right_side_opposite = 270 #angle opposite of the right sensor angle 
edge_dis = 75
deadzone = 25 #distances within this are not accurately read 

pwm = Adafruit_PCA9685.PCA9685() 

def setup():
	global  motor_change_num, current_l, sonar_dis, x_cord, y_cord, isAvoiding
        global f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar
        f_sonar = 0
        f_r_sonar = 0
        r_sonar = 0
        b_r_sonar = 0
        b_sonar = 0
        b_l_sonar = 0
        l_sonar = 0
        f_l_sonar = 0
	current_l = 0 #the drive value that the motor is using (like the joystick signals, it's between -1 and 1)
	sonar_dis = [0]*num_of_sonar
	x_cord = [0]*num_of_sonar
	y_cord = [0]*num_of_sonar
	isAvoiding = False #True when the robot is turning to edgefind
	motor_change_num = 0 #coutner for how many times the motor has changed speeds
	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pwm.set_pwm_freq(50)
	setMotors(0,0)
	
def accelerate(target_joy, turn_target): #changes the speed of the motor with a fixed max rate (determined by accel_const)
	global current_l 
	if target_joy > current_l:
		while current_l < target_joy:
              		setMotors(current_l, turn_target)
			current_l = current_l + accel_const
	if target_joy < current_l: 
                while current_l> target_joy:
			setMotors(current_l, turn_target)
                        current_l = current_l - accel_const
	current_l = target_joy
	setMotors(current_l, turn_target)

def setMotors(joy_drive, joy_turn): #sets the motors to a new PWM and prints some info 
	global motor_change_num
	new_v_drive = calcPWM(joy_drive)
	new_v_turn = calcPWM(joy_turn)
        motor_change_num = motor_change_num+1
        print ('Turn signal: %f, drive signal: %f, drive speed: %f'%(joy_turn, joy_drive, current_l)) 
	#prints (in order) the signal from the joystick for turning, the signal for driving, the value currently being used to determine drive speed
        print ('%d : Drive speed: %d Turn speed: %d' %(motor_change_num, new_v_drive, new_v_turn)) 
	#prints (in order) the motor change number, drive PWM value, turn PWM value
        pwm.set_pwm(drivePin, 0, new_v_drive)
	pwm.set_pwm(turnPin, 0, new_v_turn)
	if isAvoiding:
		time.sleep(sleep_time_edgefinding)
	else:
		time.sleep(sleep_time) #sleeps for a set amount of time

def calcPWM(joy_value): #calculates the PWM value for a joystick value, skips the range of zero PWM values unless joy_value is zero
        if joy_value > 0:
                pwm_val = int (joy_value *(drive_speed_max - drive_speed_zero_uplim) + drive_speed_zero_uplim)
        elif joy_value < 0:
                pwm_val = int (joy_value *(drive_speed_zero_lowlim - drive_speed_min) + drive_speed_zero_lowlim)
        else:
                pwm_val = drive_speed_zero
        return pwm_val

def avoidanceTurn():
	global isAvoiding
	isAvoiding = True
	if (weighted_angle_avg < right_side_low_lim )  | (weighted_angle_avg > right_side_up_lim):
		accelerate(0, edgefind_speed)
		print "turning counterclockwise"
	isAvoiding = False
	avgCord()

def motion(): #main driving function 
	print ('Driving')
	avgCord()
	if (sonar_dis[f_sonar] < tooCloseDis) | (sonar_dis[f_l_sonar] < deadzone) | (sonar_dis[f_r_sonar] < deadzone) :
		print "obstacle - stopping and turning clockwise"
 		accelerate(0,-0.5)
	elif (weighted_dis_avg > deadzone) & (weighted_dis_avg < edge_dis):
		print "edgefinding"
		avoidanceTurn() 
	else:
		print "driving"
		accelerate(1,0)

def convertCartesian():
	global x_cord, y_cord
	
	for i in range(len(sonar_dis)):
		x_cord[i] = sonar_dis[i] * math.cos(i * math.pi/4) 
		y_cord[i] = sonar_dis[i] * math.sin(i *math.pi/4)
#	print x_cord
#	print y_cord

def avgCord():
	print "finding averages..."
	global x_avg, y_avg, weighted_dis_avg, weighted_angle_avg
	x_avg = 0
	y_avg = 0
	weighted_dis_avg = 0
	weighted_angle_avg = 0
	count = 0
	angle_sum = 0
	convertCartesian() 
	for i in range(len(sonar_dis)):
		if (sonar_dis[i] < edge_dis) & (sonar_dis[i] > deadzone):
			count = count + 1
			x_avg = x_avg + x_cord[i]
			y_avg = y_avg + y_cord[i]
			angle_sum = angle_sum + i *45
	if (count != 0):
		x_avg = x_avg/count
		y_avg = y_avg/count		
		weighted_angle_avg = angle_sum / count
	else:
		weighted_angle_avg = 0
	weighted_dis_avg = math.hypot(x_avg, y_avg) 

	print weighted_angle_avg, weighted_dis_avg
	
def sonarlistener(data):
	global f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar, sonar_dis
	f_sonar = data.pins[0]
	f_r_sonar = data.pins[1]
	r_sonar = data.pins[2]
	b_r_sonar = data.pins[3]
	b_sonar = data.pins[4]
	b_l_sonar = data.pins[5]
	l_sonar = data.pins[6]
	f_l_sonar = data.pins[7]

	sonar_dis = data.distances
			
def listener():
	print ('Listening...')
	rospy.init_node('sonar_array_listener')
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



