#!/usr/bin/env python
from __future__ import division
from remyJr.msg import sensor_status
from remyJr.msg import motor_data
from remyJr.msg import pwm_val
from remyJr.msg import tof_data
from remyJr.msg import nook_status
import rospy
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

#for the motor controllers
drivePin = 0
turnPin = 1

pwm = Adafruit_PCA9685.PCA9685()

motor_control_rate = 250 # in hz
motor_queue_len = 50 #this is tied to a message type, so be careful about changing this

def setup():
	global system_ok, motor_change_num, drive_val,  turn_val, brain_command_num, last_command_num, nook_ok
        global motor_val_turn_queue, motor_val_drive_queue, motor_val_num_queue, last_last_command_num
	system_ok = False
	nook_ok = False
	drive_val = 315
	turn_val = 315
        motor_val_turn_queue = [0]*(motor_queue_len)
        motor_val_drive_queue = [0]*(motor_queue_len)
        motor_val_num_queue = [0]*(motor_queue_len)

	# tracks how many drive commands the motor control node has received
	brain_command_num = -1
	last_command_num = -1
	last_last_command_num = -2
	# this tracks how many times this node has set the motor speed
	# ideally this number should be the smae as brain_command_num
	motor_change_num = 0
	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pwm.set_pwm_freq(50)

def setMotors(joy_drive, joy_turn):
	#sets the motors to a new PWM
	global  current_l, motor_change_num, out_of_sync, current_a, last_command_num
	if (not system_ok) | (not nook_ok):
		joy_drive = 315
		joy_turn = 315
		motor_change_num +=1
	elif ((f_cliff) & (joy_drive >315)) | ((b_cliff) &(joy_drive < 315)):
		joy_drive = 315
		motor_change_num +=1
	else:
		out_of_sync = False
		motor_change_num +=1
	last_command_num = brain_command_num
      	current_l = joy_drive
	current_a = joy_turn
        pwm.set_pwm(drivePin, 0, current_l)
	pwm.set_pwm(turnPin, 0, current_a)
	publishData()

def main():
	listener()
	setMotors(drive_val, turn_val)

def moveQueue(queue, newest_entry):
        for i in range(len(queue)-1):
                queue[i] = queue[i+1]
        queue[len(queue)-1] = newest_entry
        return queue

def nookListener(data):
        #listens for the sonar data
        global nook_ok
        nook_ok = data.nook_status

def statusListener(data):
	#listens for the sonar data
	global system_ok
	system_ok = data.sensor_system_status

def pwmListener(data):
	global drive_val, turn_val, brain_command_num
	drive_val = data.drive_pwm
	turn_val = data.turn_pwm
	brain_command_num = data.pwm_command_num

def tofListener(data):
	global f_cliff, b_cliff
	f_cliff = not data.drive_front_ok
	b_cliff = not data.drive_back_ok

def listener():
	rospy.Subscriber("remyjr/nook_status", nook_status, nookListener, queue_size = 1)
	rospy.Subscriber("remyjr/pwm_val", pwm_val, pwmListener)
	rospy.Subscriber("remyjr/tof_data", tof_data, tofListener, queue_size=1)
	rospy.Subscriber("remyjr/sensor_status",  sensor_status, statusListener, queue_size=1)

def publishData():
	# publishes what the motors are currently doing including which motor command they are following
	global pub, last_last_command_num
        global motor_val_turn_queue, motor_val_drive_queue, motor_val_num_queue

	if last_command_num != last_last_command_num:
	        motor_val_turn_queue = moveQueue(motor_val_turn_queue, current_a)
	        motor_val_drive_queue = moveQueue(motor_val_drive_queue, current_l)
	        motor_val_num_queue = moveQueue(motor_val_num_queue, last_command_num)
	last_last_command_num = last_command_num
	data = motor_data()
	data.drive_val_queue = motor_val_drive_queue
	data.turn_val_queue = motor_val_turn_queue
	data.command_count_queue = motor_val_num_queue
	data.publish_count = motor_change_num
	data.last_command_count = last_command_num
	data.last_drive_val = current_l
	data.last_turn_val = current_a
	rospy.loginfo(data)
	pub.publish(data)

def mainLoop():
	rospy.init_node('robot_motors')
	global pub
	pub=rospy.Publisher('remyjr/motor_data', motor_data)
	rate = rospy.Rate(motor_control_rate)
	while not rospy.is_shutdown():
	   try:
		main() #moves
		rate.sleep()
	   except IOError, err:
		# helps prevent the program from crashing if
		# it tries to contact the featherboard in at a bad time
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



