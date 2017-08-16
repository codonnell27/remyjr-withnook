#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685


#for the motor controllers
drivePin = 0
turnPin = 1

#PWM values
drive_speed_max = 330 
drive_speed_min = 300
drive_speed_zero = 315 
drive_speed_zero_uplim = 324 #upper bound of the zero range
drive_speed_zero_lowlim = 306 #lower bound of the zero range

pwm = Adafruit_PCA9685.PCA9685() 

def setup():
	global  motor_change_num, current_l, sonar_dis, x_cord, y_cord, isAvoiding, has_jumped
	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pwm.set_pwm_freq(50)
	setMotors(0,0) #so that the robot isn't moving at startup


def setMotors(joy_drive, joy_turn): #sets the motors to a new PWM 
	global motor_change_num, current_l
	current_l = joy_drive
	new_v_drive = calcPWM(joy_drive)
	new_v_turn = calcPWM(joy_turn)
#        motor_change_num = motor_change_num+1
	print new_v_drive 
        pwm.set_pwm(drivePin, 0, new_v_drive)
	pwm.set_pwm(turnPin, 0, new_v_turn)
	time.sleep(0.1) #sleeps for a set amount of time to prevent accelerating too quickly

def calcPWM(joy_value): 
	#calculates the PWM value for a joystick value 
	#skips the range of zero PWM values unless joy_value is zero
        if joy_value > 0:
                pwm_val = int (joy_value *(drive_speed_max - drive_speed_zero_uplim) + drive_speed_zero_uplim)
		if pwm_val > drive_speed_max:
			pwm_val = drive_speed_max
        elif joy_value < 0:
                pwm_val = int (joy_value *(drive_speed_zero_lowlim - drive_speed_min) + drive_speed_zero_lowlim)
		if pwm_val < drive_speed_min:
			pwm_val = drive_speed_min
        else:
                pwm_val = drive_speed_zero
        return pwm_val


def motion(): #main driving function 
	time.sleep(10)
	setMotors(1,0)
	time.sleep(10)
	setMotors(0.5,0)

def mainLoop():
	setMotors(0.5, 0)
	time.sleep(0.25)
	while not rospy.is_shutdown():
	   try:
		motion() #moves
	   except IOError, err: #in case it tries to read the featherboard in the middle of a write
		continue
	destroy()

def destroy():
	setMotors(0,0)
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



