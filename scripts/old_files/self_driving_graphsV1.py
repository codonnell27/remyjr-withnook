#!/usr/bin/env python
from __future__ import division
from remyJr.msg import sensor_status
from remyJr.msg import imu_data
from remyJr.msg import tof_data
from std_msgs.msg import String
from remyJr.msg import sonar_array
from remyJr.msg import remyjr_graphs
from remyJr.msg import bump_data
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
drive_speed_zero_uplim = 325 #upper bound of the zero range
drive_speed_zero_lowlim = 305 #lower bound of the zero range

accel_const = 0.1 #determines the slope of the acceleration curve, must be between 0 and 1
sleep_time = 0.05 #the time the motors rest between changing PWM values

edgefind_speed = 0.35 #how fast the robot drives while edgefinding
sleep_time_edgefinding = 0.01 #the time motos rest between changing PWM values when edgefinding

obstacle_avoidance_speed = 0.1 #how fast the robot turns when avoiding obstacles

right_angle = 90 #the angle the robot tries to keep edges on
#these next two values control what the robot consideres "right" for edgefinding
# 112 & 68 make a space of ~45 degrees
right_side_width = 90 #the number of degrees the (centered around right_angle) that the robot will consider the right side 
right_side_up_lim = (right_angle + (right_side_width/2))
right_side_low_lim = (right_angle -(right_side_width/2))
right_side_opposite = (360-right_angle) #angle opposite of the right sensor angle 

edge_dis = 75 #robot ignores distances greater than this while edgefinding
deadzone = 30 #distances within this are not accurately read 
tooCloseDis = 50 #in cm -when the sonars are closer than this, the robot won't drive straight. Must be greater then 30 for accurate results

pwm = Adafruit_PCA9685.PCA9685() 

def setup():
	global  motor_change_num, current_l, sonar_dis, x_cord, y_cord, isAvoiding, immediate_danger_f, immediate_danger_b
        global f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar, tof_drive_ok_f, tof_drive_ok_b
	global avoid_direction_count, avoid_direction, sensor_ok, current_activity, heading_delta, is_turning, is_level
	global danger_prevented, f_bump, b_bump
        f_sonar = 0 #these values represent the order in which each sonar's data is sent. 
        f_r_sonar = 1 
        r_sonar = 2
        b_r_sonar = 3
        b_sonar = 4
        b_l_sonar = 5
        l_sonar = 6
        f_l_sonar = 7
	#these variables are used to determine which direction the robot turns when avoiding obstacles
	avoid_direction_count = 0 #this is a count to ensure that the robot varies which direction it turns when avoiding obstacles
	avoid_direction = 1 #this indicates the direction the robot will turn when avoiding obstacles
	tof_drive_ok_f = False #if true, there is a cliff in front of the robot
	tof_drive_ok_b = False #if true, there is a cliff behind the robot
	danger_prevented = False #true when the SetMotors function prevents the robot from moving due to immediate obstacles
	heading_delta = 0 
	b_bump = False
	f_bump = False
	immediate_danger_f = False #true when there is a close object or a cliff in front
	immediate_danger_b = False
	is_turning = False #true when the imu thinks the robot is turning
	is_level = True #true indicates that the robot is not level (usually when it tries to drive into something)
	sensor_ok = False #false indicates that not all sensors are online and working
	current_activity = 'starting up' 
	current_l = 0 #the drive value that the motor is using (like the joystick signals, it's between -1 and 1)
	sonar_dis = [0]*num_of_sonar #holds the distances for each sonar
	x_cord = [0]*num_of_sonar #holds the x-cordinates for objects detected by each sonar
	y_cord = [0]*num_of_sonar #holds the y-cordinates for objects deteced by each sonar
	avgCord()
	isAvoiding = False #True when the robot is turning to edgefind. used to control how long the robot sleeps between changing pwm values
	motor_change_num = 0 #counter for how many times the motor has changed speeds
	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pwm.set_pwm_freq(50)
	setMotors(0,0) #so that the robot isn't moving at startup

def driveStraight(): 
	#since the caster wheel determines whether the robot is actually driving straight, this helps correct for that 
	if not is_turning:
		return 0
	else:
		return (-0.1*heading_delta)
		
def accelerate(drive_target, turn_target): 
	#changes the speed of the motor with a fixed max rate (determined by accel_const)
	#only changes the forward/back velocity gradually, not turn velocity, except when it thinks it's going straight but it's not
	global current_l, danger_prevented
	if (turn_target ==0) & (not isAvoiding):
		turn_target = current_l*driveStraight() 
	if drive_target > current_l:
		while (current_l < drive_target) & (not danger_prevented):
              		setMotors(current_l, turn_target)
			current_l = current_l + accel_const
	elif drive_target < current_l: 
                while (current_l> drive_target) & (not danger_prevented):
			setMotors(current_l, turn_target)
                        current_l = current_l - accel_const
	if not danger_prevented:
		current_l = drive_target
	danger_prevented = False
	setMotors(current_l, turn_target)

def setMotors(joy_drive, joy_turn): #sets the motors to a new PWM 
	global motor_change_num, current_l, current_activity, danger_prevented
	#these next lines prevent the robot from driving backward or forward when there is an immediate danger
	findDangers()
	print "immediate dangers ", immediate_danger_f, immediate_danger_b
	if not sensor_ok:
		joy_drive = 0
		jou_turn = 0
		danger_prevented = True
		current_activity = "Waiting for all sensors to come back online"
	elif (joy_drive > 0) & (immediate_danger_f):
		joy_drive = 0
		danger_prevented = True
		current_activity = "Danger detected in front - prevented from moving forward"
	elif (joy_drive < 0) & (immediate_danger_b):
		joy_drive = 0
		danger_prevented = True
		current_activity = "Danger detected behind - prevented from driving back"
      	current_l = joy_drive
	new_v_drive = calcPWM(joy_drive)
	new_v_turn = calcPWM(joy_turn)
        motor_change_num = motor_change_num+1 
        pwm.set_pwm(drivePin, 0, new_v_drive)
	pwm.set_pwm(turnPin, 0, new_v_turn)
	if isAvoiding: 
		#because sleep_time_edgefinding is a smaller value than sleep_time
		#this helps prevent the robot from overshooting its turn
		time.sleep(sleep_time_edgefinding)
	else:
		time.sleep(sleep_time) #sleeps for a set amount of time to prevent accelerating too quickly

def calcPWM(joy_value):
	#calculates the PWM value for a joystick value (between -1 and 1)
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

def edgefindTurn():
	#decides whether the robot is facing the right direction and which direction to turn if it's not
	global isAvoiding, current_activity
	isAvoiding = True #so that the pause between setting pwm values is shorter to avoid overshooting it's turn
	if (weighted_angle_avg < right_side_low_lim )  | (weighted_angle_avg > right_side_opposite):
		accelerate(0, edgefind_speed)
		current_activity = "edgefinding - turning %f" %edgefind_speed
	elif (weighted_angle_avg <= right_side_opposite )  & (weighted_angle_avg > right_side_up_lim):
                accelerate(0, -1*edgefind_speed)
                current_activity = "edgefinding - turning %f" %(-1*edgefind_speed)
	else:
		current_activity =  "Found edge!!! - driving"
		accelerate(1,0)
	isAvoiding = False

def reverse(go_in_direction): #more of a reverse and turn
	#if the robot needs to back up and find a new course
	#go_in_direction is -1 if it needs to back up (obstacle in front)
	#or 1 if it needs to go straight (obstacle behind)
	global isAvoiding, current_activity, avoidance_count
	isAvoiding = True
	c_l = current_l
	if (c_l == 0) & (not is_level):
		c_l = 0.5
	if avoidance_count >= 5:
		c_l = 1
		current_activity= "unable to turn successfully, reversing"
		avoidance_count = 0
	accelerate(go_in_direction*abs(c_l), 0)
	time.sleep(0.2)
	accelerate(0,avoid_direction*obstacle_avoidance_speed)
	updateAvoidDirection()
	time.sleep(0.5)
	isAvoiding = False

def updateAvoidDirection():
	#every once in a while, this switches the direction it turns when avoiding obstacles
	# this helps prevent the robot from getting suck
	#avoid_direction is +1 when it turns clockwise and -1 when it turns counterclockwise
	global avoid_direction, avoid_direction_count, avoidance_count
	if avoid_direction_count >= 15:
		avoid_direction = -avoid_direction
		avoid_direction_count = 0
		avoidance_count +=1
	else:
		avoid_direction_count +=1 

def closeQuarters():
	#determines how open the space it's in is 
	#i.e. middle of desk spaces or not
	# the smaller the value of how_close the more confined the area
	how_close = 0
	for i in range(len(sonar_dis)):
		if (sonar_dis[i] > deadzone) & (sonar_dis[i] < edge_dis):
			if sonar_dis > tooCloseDis:
				how_close +=1
			else:
				how_close += 0.5
	how_close = how_close /len(sonar_dis)	
	return how_close

def findDangers():
	#determines if there are an immediate dangers (cliff, object) in front or behind
	global current_activity
	global immediate_danger_f, immediate_danger_b
	if (not tof_drive_ok_f) | (sonar_dis[f_sonar] < deadzone +5) | f_bump:  #  |(sonar_dis[f_l_sonar] < deadzone) | (sonar_dis[f_r_sonar] < deadzone):
 		immediate_danger_f = True
	else:
		immediate_danger_f = False
        if (not tof_drive_ok_b) | (sonar_dis[b_sonar] < deadzone -5) | b_bump:  #  |(sonar_dis[f_l_sonar] < deadzone) | (sonar_dis[f_r_sonar] < deadzone):
                immediate_danger_b = True
        else:
                immediate_danger_b = False

def motion(): #main driving function 
	global current_activity, avoidance_count
	current_activity = "calculating motion"
	print b_bump, f_bump
	if not sensor_ok: #one or more of the sensors are down, so it stops and waits for the sensors to come online again
		accelerate(0,0)
		avoidance_count = 0
		current_activity  = "waiting for all sensors to come online again"
	elif ((not tof_drive_ok_f) & (not tof_drive_ok_b)): #stops and turns if there are cliffs in front and back
		setMotors(0,0)
		reverse(0)
		current_activity = "cliff detected in front and behind - %f" % (avoid_direction*obstacle_avoidance_speed)
	elif ((not tof_drive_ok_f)| (f_bump)) | (not is_level): #the makes sure that the robot no longer being level doesn't trigger this.
		#stops, backs up, and turns if there is a cliff in front
		setMotors(0,0)
		reverse(-1)
		if not tof_drive_ok_f:
			current_activity = "cliff detected in front  reversing %f" % (avoid_direction*obstacle_avoidance_speed)
		elif f_bump:
			current_activity = "robot hit something in front, reversing %f" % (avoid_direction*obstacle_avoidance_speed)
		else:
			current_activity = "no longer level - reversing %f" % (avoid_direction*obstacle_avoidance_speed)
	elif ((not tof_drive_ok_b) | (b_bump)): #if there is a cliff behind, it stops, goes forward, and turns
		setMotors(0,0)
		reverse(1)
		if not tof_drive_ok_b:
			current_activity = "cliff detected behind, driving forward %f" % (avoid_direction*obstacle_avoidance_speed)
		else:
			current_activity = "robot hit something in back, driving forward %f" % (avoid_direction*obstacle_avoidance_speed)
	elif (sonar_dis[f_sonar] < tooCloseDis): #(sonar_dis[f_l_sonar] < deadzone + 5) | (sonar_dis[f_r_sonar] < deadzone+5):
		#when there is something in front of the robot or it is no longer level, it stops and turns
		current_activity = "obstacle - stopping and turning %f" % (avoid_direction*obstacle_avoidance_speed)
		how_close = closeQuarters()
		reverse(-1*how_close)
	elif (weighted_dis_avg > deadzone) & (weighted_dis_avg < edge_dis):  
		edgefindTurn() 
                avoidance_count = 0
	else: #when none of the sensors are reporting an immediate concern
		current_activity = "driving"
                avoidance_count = 0
		accelerate(1,0)

def convertCartesian():
	#converts sonar data to cartesian coordinates
	# front sonar is zero degrees (assuming data is received from sonar_array node 
	#in counterclockwise order beginning from the front sonar
	global x_cord, y_cord
	for i in range(len(sonar_dis)):
		x_cord[i] = sonar_dis[i] * math.sin(i * math.pi/4) 
		y_cord[i] = sonar_dis[i] * math.cos(i *math.pi/4)

def avgCord():
	#calculates the weighted average of the sonar data for edgefinding
	global x_avg, y_avg, weighted_dis_avg, weighted_angle_avg
	x_avg = 0 #hold the cartesian coordinates for each sonar's data
	y_avg = 0
	#these two points are polar coordinates of the average of sonar data within a certain radius band
	weighted_dis_avg = 0 #the distance of an average of the points within a certain radius range
	weighted_angle_avg = 0 #the angle of an average of the points within that certain radius range
	count = 0
	angle_sum = 0
	convertCartesian() 
	for i in range(len(sonar_dis)):
		if (sonar_dis[i] < edge_dis) & (sonar_dis[i] > deadzone):
			weight = (edge_dis - sonar_dis[i]) **2.5
			count = count + weight
			x_avg = x_avg + x_cord[i]*(weight)
 			y_avg = y_avg + y_cord[i]*(weight)
			angle_sum = angle_sum + i *45*(weight)
	if (count != 0):
		x_avg = x_avg/count
		y_avg = y_avg/count		
		weighted_angle_avg = angle_sum / count
	else:
		weighted_angle_avg = 0
	weighted_dis_avg = math.hypot(x_avg, y_avg)
	
def sonarlistener(data): #listens for the sonar data
	global sonar_dis
	avgCord()
	sonar_dis = data.distances

def imuListener(data): #the imu checks whether the robot is level and whether the robot is going straight
	global is_level, is_turning, heading_delta
	is_level = data.is_level
	is_turning = data.is_turning
	heading_delta = data.heading_delta   	

def sensorStatusListener(data): #this node checks whether the sensors are online and reports True if all sensors are working
	global sensor_ok
	sensor_ok = data.sensor_system_status
	
def tofListener(data): #the time of flight sensor checks to see if there is a cliff/suddent drop of floor in front or behind the robot
	global tof_drive_ok_f, tof_drive_ok_b
	tof_drive_ok_f = data.drive_front_ok
	tof_drive_ok_b = data.drive_back_ok

def bumpSkirtListener(data):
	global f_bump, b_bump
	f_bump = data.f_bump
	b_bump = data.b_bump
	
def listener():
	rospy.Subscriber("remyjr/imu_data", imu_data, imuListener, queue_size = 1)
	rospy.Subscriber("remyjr/sonar_data", sonar_array, sonarlistener, queue_size = 1)
	rospy.Subscriber("remyjr/sensor_status", sensor_status, sensorStatusListener, queue_size = 1)	
	rospy.Subscriber("remyjr/tof_data", tof_data, tofListener, queue_size=1)
	rospy.Subscriber("remyjr/bump_data", bump_data, bumpSkirtListener, queue_size=1)
	
def publishData(): #sends the data to the graphing node and current_activity node
	global pub1, pub2
	motion_data = remyjr_graphs()
	motion_data.deadzone = deadzone
	motion_data.tooCloseDis = tooCloseDis
	motion_data.edgeDis = edge_dis
	motion_data.x_cords = x_cord
	motion_data.y_cords = y_cord
	motion_data.weighted_dis_avg = weighted_dis_avg
	motion_data.weighted_angle_avg = weighted_angle_avg
	motion_data.x_avg = x_avg
	motion_data.y_avg = y_avg
	pub1.publish(motion_data)
	activity = String()
	activity.data = current_activity
	pub2.publish(activity)

def mainLoop():
	rospy.init_node('robot_controller')
	global pub1, pub2, pub3 
	pub1=rospy.Publisher('remyjr/motion_data', remyjr_graphs, queue_size=1) 
	pub2=rospy.Publisher('remyjr/current_activity', String, queue_size=1)
	while not rospy.is_shutdown():
	   try:
		listener() #listens to sonar data
		motion() #moves
		publishData() #sends data to graphing node
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



