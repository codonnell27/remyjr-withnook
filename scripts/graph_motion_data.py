#!/usr/bin/env python
from remyJr.msg import remyjr_graphs
from remyJr.msg import motor_data
from remyJr.msg import pwm_val
import rospy
import time
import math
import matplotlib.pyplot as plt
import numpy as np

#holds the x and y coordinates of unit vectors representing each direction a sonar is facing
sonar_x_cords = [0.0, 1/1.4142, 1.0, 1/1.4142, 0.0, -1/1.4142, -1.0, -1/1.4142, 0.0]
sonar_y_cords = [1.0, 1/1.4142, 0.0, -1/1.4142, -1.0, -1/1.4142, 0.0, 1/1.4142, 1.0]

# this value must be the same as the length of the distances array in sonar_array.msg
num_of_sonar = 8

# controls how many close quarters values are stored and displayed
close_quarters_queue_len = 40
close_quarters_x_vals = np.arange(close_quarters_queue_len)

# this value must be the same as the length of the arrays that hold the
# pwm commands in motor_data.msg and pwm_val.msg
motor_queue_len = 50

def setup():
	global sonar_dis, x_cord, y_cord, x_avg, y_avg, edgeDis, deadzone, tooCloseDis, pwm_zero_min
	global close_quarters_queue, close_quarters, close_quarters_edgefinding_lim, pwm_zero_max
        global pwm_command_drive_queue, pwm_command_turn_queue, pwm_command_num_queue, pwm_zero_max_y
        global motor_val_turn_queue, motor_val_drive_queue, motor_val_num_queue, pwm_max, pwm_min, pwm_zero_min_y

	# these next arrays hold values used to graph  the brain pwm commands vs
	# what the motor driver are following
	motor_val_turn_queue = np.zeros(motor_queue_len)
	motor_val_drive_queue = np.zeros(motor_queue_len)
	motor_val_num_queue = np.zeros(motor_queue_len)
        pwm_command_turn_queue = np.zeros(motor_queue_len)
        pwm_command_drive_queue = np.zeros(motor_queue_len)
        pwm_command_num_queue = np.zeros(motor_queue_len)

	# these variables hold values used to visualize how tight the space the robot is in
	close_quarters_edgefinding_lim = 0
	close_quarters = 0
	close_quarters_queue = np.zeros(close_quarters_queue_len)
	sonar_dis = [0]*num_of_sonar #holds the distances for each sonar
	x_cord = [0]*num_of_sonar #holds the x-cordinates for objects detected by each sonar
	y_cord = [0]*num_of_sonar #holds the y-cordinates for objects deteced by each sonar
	x_avg = 0 #weighted average of the x-coordinates of the sonar data, received from another node
	y_avg = 0 #weighted average of the y-coordinates of the sonar data, received from another node
	#these are all received from another node and represent the radius of different zones used in navigating
	edgeDis = 0
	deadzone = 0
	tooCloseDis = 0
	pwm_max = 0
	pwm_min = 0
	pwm_zero_max = 0
	pwm_zero_min = 0
	pwm_zero_min_y = np.zeros(motor_queue_len)
	pwm_zero_max_y = np.zeros(motor_queue_len)
	plt.ion()
        fig = plt.figure(figsize=(65,30), dpi=45)
	findRings() #calculates the coordinates that represent the different zones used in navigating

def moveQueue(queue, newest_entry):
	for i in range(len(queue)-1):
		queue[i] = queue[i+1]
 	queue[len(queue)-1] = newest_entry
	return queue

def getRingCordX(dis): #calcluates the x-cordinates representing a radius around the robot
	cord = [0]*(num_of_sonar+1)
	for i in range(len(cord)):
		cord[i] = dis * sonar_x_cords[i]
	return cord

def getRingCordY(dis): #calculates the y-coordinates representing a radius around the robot
        cord = [0]*(num_of_sonar + 1)
        for i in range(len(cord)):
                cord[i] = dis * sonar_y_cords[i]
        return cord

def findRings(): #calculates the coordiates representing various regions around the robot
	global deadzone_x, deadzone_y, tooClose_x, pwm_zero_min_y, tooClose_y, edge_x, edge_y, edgefinding_lim_y, pwm_zero_max_y
	deadzone_x = getRingCordX(deadzone)
        deadzone_y = getRingCordY(deadzone)
	tooClose_x = getRingCordX(tooCloseDis)
        tooClose_y = getRingCordY(tooCloseDis)
        edge_x = getRingCordX( edgeDis)
        edge_y = getRingCordY(edgeDis)
	pwm_zero_max_y = np.ones(motor_queue_len)*(pwm_zero_max -1)
	pwm_zero_min_y = np.ones(motor_queue_len)*(pwm_zero_min+1)
	edgefinding_lim_y = np.ones(close_quarters_queue_len)*close_quarters_edgefinding_lim

def plotData(): #plots the data
	# this updates the close quarters values stored in close_quarters_queue
        global close_quarters_queue
	close_quarters_queue = moveQueue(close_quarters_queue, close_quarters)

	plt.clf()

	# graphs the close quarters value, which represents how tight a space the robot is in
	# if this value is above the edgefinding threshold, the robot will edgefind
	# if it's below, the robot will just pick the direction with the furthest object and drive towards that
        plt.subplot2grid((14,2),(0,0), rowspan=4, colspan=1)
        plt.ylim(0,1)
        plt.plot(close_quarters_x_vals, edgefinding_lim_y, color="red", linewidth=5, label="Edgefinging Threshold")
        plt.plot(close_quarters_x_vals, close_quarters_queue, color="blue", linewidth =5, label="Close Quarters Value")
	plt.ylabel("Close Quarters Value", fontsize=30)
	plt.title("How Tight a Space the Robot is in", fontsize=30)
	plt.legend(loc='upper left', fontsize=20)

	# graphs the drive pwm values
        # this show the current pwm commands that the brain node sent vs those that the motor driver is following
        # which allows us to easily see if the motor driver is lagging significantly or missing commands
        plt.subplot2grid((14,2), (5,0), rowspan=4, colspan=1)
        plt.xlim(pwm_command_num_queue[0], pwm_command_num_queue[motor_queue_len -1]+2)
        plt.ylim(pwm_min -5,pwm_max+5)
	plt.ylabel("PWM Values", fontsize=30)
	plt.xlabel("Motor Driver Commands", fontsize=30)
        plt.text(pwm_command_num_queue[40], pwm_min, "Forward", fontsize=20)
        plt.text(pwm_command_num_queue[40], pwm_max, "Backward", fontsize=20)
	plt.title("Drive PWM Values - Latest Brain Commands vs Latest Motor Driver Commands", fontsize=30)
	plt.plot(pwm_command_num_queue, pwm_zero_max_y, color="gray", linewidth=3, label="Zero Range Maximum and Minimum")
        plt.plot(pwm_command_num_queue, pwm_zero_min_y, color="gray", linewidth=3)
        plt.plot(pwm_command_num_queue, pwm_command_drive_queue, color="red", linewidth=12, label="Brain Commands")
        plt.plot(motor_val_num_queue, motor_val_drive_queue, color="black", linewidth=4, label="Motor Driver Commands")
	plt.legend(loc='upper left', fontsize=20)

	# graphs the turn pwm values
	# this show the current pwm commands that the brain node sent vs those that the motor driver is following
	# which allows us to easily see if the motor driver is lagging significantly or missing commands
        plt.subplot2grid((14,2), (10,0), rowspan=4, colspan=1)
        plt.xlim(pwm_command_num_queue[0], pwm_command_num_queue[motor_queue_len -1] + 2)
        plt.ylim(pwm_min -5,pwm_max + 5)
        plt.ylabel("PWM Values", fontsize=30)
        plt.xlabel("Command Number", fontsize=30)
	plt.text(pwm_command_num_queue[40], pwm_min, "Clockwise", fontsize=20)
	plt.text(pwm_command_num_queue[39], pwm_max, "Counterclockwise", fontsize=20)
        plt.title("Turn PWM Values - Latest Brain Commands vs. Latest Motor Driver Commands", fontsize=30)
        plt.plot(pwm_command_num_queue, pwm_zero_max_y, color="gray", linewidth=3, label="Zero Range Maximum and Minimum")
        plt.plot(pwm_command_num_queue, pwm_zero_min_y, color="gray", linewidth=3)
        plt.plot(pwm_command_num_queue, pwm_command_turn_queue, color="red", linewidth=12, label="Brain Commands")
        plt.plot(motor_val_num_queue, motor_val_turn_queue, color="black", linewidth=4, label="Motor Driver Commands")
	plt.legend(loc='upper left', fontsize=20)

	# graphs the sonar data
	# also graphs octagons representing various regions the robot uses in navigating
	# i.e. deadzone, too close zone, edgezone
	# it also graphs the weighted average of the sonar data - this is the point
	# that the robot tries to keep on it's right while edgefinding
	plt.subplot2grid((14,2), (0,1), rowspan=16, colspan=1)
        plt.xlim(-1*edgeDis - 30.0, edgeDis + 30.0)
        plt.ylim(-1*edgeDis - 30.0, edgeDis + 30.0)
        plt.title("Sonar Data - Birds Eye View", fontsize=30)
	plt.text(-deadzone/2, 0,'Center of Sonar Array', fontsize=20)
	plt.plot(deadzone_x, deadzone_y, color="red", linewidth=5, label="Deadzone Radius")
	plt.plot([x_cord[0], 0], [y_cord[0], 0], color="green", linewidth=2, label="Front of Robot")
	plt.plot(tooClose_x, tooClose_y, color="yellow", linewidth=8, label="Too Close Radius")
	plt.plot(edge_x, edge_y, color="green", linewidth=5, label="Ignores everything outside this radius while edgefinding")
	plt.scatter(x_cord, y_cord, color="blue", s=500, marker='o', label="Objects Seen by Sonar")
	plt.scatter(x_avg, y_avg, color="red", s=750, marker='o', label="Average Object Distance for Edgefinding")
	plt.legend(loc='upper left', fontsize=20)

	plt.draw()
	plt.pause(0.001) # the graphs are blank without this pause

def motionDataListener(data):
	global x_cord, y_cord, x_avg, y_avg,close_quarters, deadzone, tooCloseDis, edgeDis, close_quarters_edgefinding_lim
	global close_quarters_queue, pwm_max, pwm_min, pwm_zero_max, pwm_zero_min
	x_cord = data.x_cords
	y_cord = data.y_cords
	x_avg = data.x_avg
	y_avg = data.y_avg

	# these next values are the same for every message, but they are used to scale graphs
	# and draw references on those graphs
	close_quarters = data.close_quarters
	deadzone = data.deadzone
	tooCloseDis = data.tooCloseDis
	edgeDis = data.edgeDis
	pwm_max = data.pwm_max
	pwm_min = data.pwm_min
	pwm_zero_min = data.pwm_zero_min
	pwm_zero_max = data.pwm_zero_max
	close_quarters_edgefinding_lim = data.close_quarters_edgefinding_lim

	findRings()

def motorListener(data):
	# listens for the arrays that hold the previous motor driver pwm values
	global motor_val_turn_queue, motor_val_drive_queue, motor_val_num_queue
	motor_val_turn_queue = data.turn_val_queue
	motor_val_drive_queue = data.drive_val_queue
	motor_val_num_queue = data.command_count_queue

def pwmCommandListener(data):
	# listens for the arrays that hold the previous brain pwm commands
	global pwm_command_drive_queue, pwm_command_turn_queue, pwm_command_num_queue
	pwm_command_drive_queue = data.drive_pwm_queue
	pwm_command_turn_queue = data.turn_pwm_queue
	pwm_command_num_queue = data.pwm_command_num_queue

def listener():
	rospy.Subscriber("remyjr/motion_data", remyjr_graphs, motionDataListener, queue_size = 1)
	rospy.Subscriber("remyjr/motor_data", motor_data, motorListener, queue_size=1)
	rospy.Subscriber("remyjr/pwm_val", pwm_val, pwmCommandListener, queue_size=1)

def mainLoop():
	setup()
	rospy.init_node('remyjr_graphs')
	while not rospy.is_shutdown():
		listener()
		plotData()

if __name__ == "__main__":
        try:
		mainLoop()
        except KeyboardInterrupt:
		pass



