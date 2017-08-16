#!/usr/bin/env python
from __future__ import division
import rospy
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()


drive_speed_max = 330
drive_speed_min = 300
drive_speed_zero = 315
drive_speed_zero_uplim = 324 #upper bound of the zero range
drive_speed_zero_lowlim = 306 #lower bound of the zero range
#for the motor controllers
drivePin = 0
turnPin = 1




def setMotors(joy_drive, joy_turn): #sets the motors to a new PWM and prints some info 
        global motor_change_num
        new_v_drive = calcPWM(joy_drive)
        new_v_turn = calcPWM(joy_turn)
#        motor_change_num = motor_change_num+1
        #print ('Turn signal: %f, drive signal: %f, drive speed: %f'%(joy_turn, joy_drive, current_l))  
        #print ('%d : Drive speed: %d Turn speed: %d' %(motor_change_num, new_v_drive, new_v_turn))     
        pwm.set_pwm(drivePin, 0, new_v_drive)
        pwm.set_pwm(turnPin, 0, new_v_turn)

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

def setup():
	global joy_val
	joy_val = drive_speed_zero
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pwm.set_pwm_freq(50)
        setMotors(0,0) #so that the robot isn't moving at startup

def motion():
	global joy_val
	print joy_val
	pwm.set_pwm(turnPin, 0, joy_val)
	time.sleep(0.25)
	setMotors(0, 0)
	time.sleep(0.2)
	joy_val += 1

def mainLoop():
        while not rospy.is_shutdown():
		while joy_val < drive_speed_max:
                	motion() #moves
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

