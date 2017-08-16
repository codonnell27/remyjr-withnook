#!/usr/bin/env python
from __future__ import division
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import rospy
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

TRIG = 18
ECHO = 17

drivePin = 0
turnPin = 1

joystick_linear_axis = 1
joystick_angular_axis = 2


drive_speed_max = 330
drive_speed_min = 300
drive_speed_zero = 315

tooCloseDis = 45

pwm = Adafruit_PCA9685.PCA9685()

global v_l_previous 

def setup():
	global v_l_previous

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

	v_l_previous = drive_speed_zero
        pwm.set_pwm_freq(50)
        time.sleep(20)

def incrementSpeed(final_v, previous_v):
	v_step= int ( (final_v - previous_v)/2 + previous_v)
	return v_step

def motion(v_l, v_a):
#	global v_l_previous
#	v_l_initial = incrementSpeed(v_l, v_l_previous)
	if v_a != drive_speed_zero:
		pwm.set_pwm(drivePin, 0, drive_speed_zero)
		pwm.set_pwm(turnPin, 0, v_a)
#		v_l_previous = drive_speed_zero

	elif (v_l != drive_speed_zero) & distance():
		pwm.set_pwm(turnPin, 0, drive_speed_zero)
		pwm.set_pwm(drivePin, 0, v_l)
#		v_l_previous = v_l
	else:
		pwm.set_pwm(drivePin, 0, drive_speed_zero)
		pwm.set_pwm(turnPin, 0, drive_speed_zero)
#		v_l_previous = drive_speed_zero

def callback(data):

	twist = Twist()
#	rospy.loginfo("recieved a /cmd_vel message!")
#	rospy.loginfo("linear components: [%f, %f, %f]") %(data.linear.x, data.linear.y, data.linear.z)
#	rospy.loginfo("angular components: [%f, %f, %f]") %(data.angular.x, data.angular.y, data.angular.z)
	
	v_l = data.linear.x *(drive_speed_max - drive_speed_min)/2 + drive_speed_zero
	v_a = data.angular.z *(drive_speed_max - drive_speed_min)/2 + drive_speed_zero
	

	motion(int(v_l), int(v_a))

	if (v_l != drive_speed_zero) & distance():
		
		twist.linear.x = v_l
		twist.angular.z = drive_speed_zero

	elif v_a != drive_speed_zero:
		
		twist.linear.x = drive_speed_zero
		twist.angular.z= v_a
	else:
		
		twist.linear.x = drive_speed_zero
		twist.angular.z = drive_speed_zero


	pub.publish(twist)

def distance():
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

	notTooClose = False

	if dis > tooCloseDis:
		notTooClose = True
	
	return notTooClose


def listener():
	global pub
	pub = rospy.Publisher('remyjr/pwm_value', Twist)
	rospy.init_node('cmd_vel_listener')
	rospy.Subscriber("remyjr/cmd_vel", Twist, callback)
	
	rospy.spin() 
          

def destroy():
        GPIO.cleanup()

if __name__ == "__main__":
        setup()
        try:
                listener()
        except rospy.ROSInterruptException:
                destroy()
		pass



