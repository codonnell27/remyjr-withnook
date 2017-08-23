#!/usr/bin/env python
from remyJr.msg import graphics_queues
from remyJr.msg import motor_data
from remyJr.msg import pwm_val
from remyJr.msg import graphics_data
import rospy
import numpy as np
<<<<<<< HEAD
node_rate = 52  #in Hz
=======
node_rate = 32  #in Hz
>>>>>>> 3fbebfb537b65e7f0f3b5badf7b4b8d02564bc61
motor_queue_len = 100


def setup():
        global close_quarters_queue, last_brain_command_num, last_motor_command_num, edgefinding_lim_y
        global pwm_command_drive_queue, pwm_command_turn_queue, pwm_command_num_queue, close_quarters
        global motor_val_turn_queue, motor_val_drive_queue, motor_val_num_queue

	last_brain_command_num = 0
	last_motor_command_num = 0
	close_quarters = 0

	# these are used to graph close_quarters over time, which gives us a sense of
	# how tight a space the robot is in over time
        close_quarters_queue = np.zeros(motor_queue_len)
	edgefinding_lim_y = np.zeros(motor_queue_len)

        # these next arrays hold values used to graph  the brain pwm commands vs
        # what the motor driver are following
        motor_val_turn_queue = np.zeros(motor_queue_len)
        motor_val_drive_queue = np.zeros(motor_queue_len)
        motor_val_num_queue = np.zeros(motor_queue_len)
        pwm_command_turn_queue = np.zeros(motor_queue_len)
        pwm_command_drive_queue = np.zeros(motor_queue_len)
        pwm_command_num_queue = np.zeros(motor_queue_len)

def moveQueue(queue, newest_entry):
        for i in range(len(queue)-1):
                queue[i] = queue[i+1]
        queue[len(queue)-1] = newest_entry
        return queue

def motionDataListener(data):
        global edgefinding_lim_y
        global close_quarters
        close_quarters = data.close_quarters
        edgefinding_lim_y = np.ones(motor_queue_len)*data.close_quarters_edgefinding_lim

def motorListener(data):
	global motor_val_turn_queue, motor_val_drive_queue, motor_val_num_queue, last_motor_command_num
	if last_motor_command_num != data.last_command_count:
		# updates the queues when there is  new data
		motor_val_turn_queue = moveQueue(motor_val_turn_queue, data.last_turn_val)
		motor_val_drive_queue = moveQueue(motor_val_drive_queue, data.last_drive_val)
		motor_val_num_queue = moveQueue(motor_val_num_queue, data.last_command_count)
	last_motor_command_num = data.last_command_count

def pwmCommandListener(data):
	global pwm_command_drive_queue, pwm_command_turn_queue, pwm_command_num_queue, last_brain_command_num
	global close_quarters_queue
	if last_brain_command_num != data.pwm_command_num:
		# updates the queues to account for new data
		pwm_command_drive_queue = moveQueue(pwm_command_drive_queue, data.drive_pwm)
		pwm_command_turn_queue = moveQueue(pwm_command_turn_queue, data.turn_pwm)
		pwm_command_num_queue = moveQueue(pwm_command_num_queue, data.pwm_command_num)
	        close_quarters_queue = moveQueue(close_quarters_queue, close_quarters)
	last_brain_command_num = data.pwm_command_num

def listener():
	rospy.Subscriber("remyjr/motion_data", graphics_data, motionDataListener)
        rospy.Subscriber("remyjr/motor_data", motor_data, motorListener, queue_size = 40)
        rospy.Subscriber("remyjr/pwm_val", pwm_val, pwmCommandListener, queue_size = 40)

def publish():
	data= graphics_queues()
	data.close_quarters_queue = close_quarters_queue
	data.edgefinding_lim_y = edgefinding_lim_y
	data.drive_pwm_queue = pwm_command_drive_queue
	data.turn_pwm_queue = pwm_command_turn_queue
	data.pwm_command_num_queue = pwm_command_num_queue
	data.motor_val_drive_queue = motor_val_drive_queue
	data.motor_val_turn_queue = motor_val_turn_queue
	data.motor_val_num_queue = motor_val_num_queue
	pub.publish(data)

def mainLoop():
        rospy.init_node('graphics_queue')
        global pub
        pub=rospy.Publisher('remyjr/graphics_queues', graphics_queues, queue_size = 2)
        rate = rospy.Rate(node_rate)
        while not rospy.is_shutdown():
                listener()
		publish()
                rate.sleep()

if __name__ == "__main__":
        setup()
        try:
                mainLoop()
        except KeyboardInterrupt:
                pass





