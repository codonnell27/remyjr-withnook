#!/usr/bin/env python
from __future__ import division
from remyJr.msg import sensor_status
from remyJr.msg import imu_data
from remyJr.msg import tof_data
from std_msgs.msg import String
from std_msgs.msg import Bool
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
# upper and lower bounds of the zero range
# these correspond to the lowest PWM values 
# that result in the robot acutally driving
# for some reason these don't result in the same speed??? 
drive_speed_zero_uplim = 325 
drive_speed_zero_lowlim = 305 

# together these control how fast the robot accelerates
# accel_const determines the slope of the acceleration curve 
# it must be between 0 and 1
# sleep_time (in seconds) determines the time the motors rest between changing PWM values
accel_const = 0.1 
sleep_time = 0.05

#determines how fast the robot turns when edgefinding
# edgefind_speed must be between -1 and 1 (a negative number
# will result in the robot turning in the opposite direction)
# sleep_time_edgefinding determines how long the robot sleeps
# between changing PWM values while edgefinding
edgefind_speed = 0.35 
sleep_time_edgefinding = 0.01 

# how fast the robot turns when avoiding obstacles
# a smaller value is usually better to avoid overshooting 
obstacle_avoidance_speed = 0.1 

# the angle the robot tries to keep edges on while edgefinding
right_angle = 90
# these next two values control what the robot considers to be within 
# an acceptable range of right_angle
# right_side_width is the number of degrees (centered around right_angle)
# that the robot will consider to be ok when edgefinding
# i.e. when the "edge" is within that range, the robot will drive straight 
# instead of turning
right_side_width = 90  
right_side_up_lim = (right_angle + (right_side_width/2))
right_side_low_lim = (right_angle -(right_side_width/2))
right_side_opposite = (360-right_angle)  

# the robot ignores distances greater than this while edgefinding (in cm)
edge_dis = 100 
# distances within this (in cm, so ~1 ft) are not accurately read by the sonar
deadzone = 30 
# approx. distance (in cm) that the front sticks out from the front sonar 
front_distance = 45
# when the front sonar reads a distance closer than this (in cm)
# the robot won't drive straight. Must be greater then 30cm for accurate reading
tooCloseDis = 60 

# this value is compared to how_close to determine if the space is too crowded to edgefind
# how_close is a value (calculated later) between 0 and 1
# 0 being that the robot is in a very confined space and 1 being 
# that all sonar are reading a distance greater than edge_dis
# when how_close is less than close_quarters_edgefinding_lim then the robot won't edgefind
# it will just drive straight until it finds and obstacle or it is no longer in such a 
# confined space 
close_quarters_edgefinding_lim = 0.5

pwm = Adafruit_PCA9685.PCA9685() 

def setup():
	global  motor_change_num, current_l, sonar_dis, x_cord, y_cord, isAvoiding, immediate_danger_f, immediate_danger_b
        global f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar, tof_drive_ok_f, tof_drive_ok_b
	global avoid_direction_count, avoid_direction, sensor_ok, current_activity, heading_delta,  is_level
	global danger_prevented, f_bump, b_bump, terrible_mistake
	terrible_mistake = False
	#these values represent the order in which seach sonar's data is sent
        f_sonar = 0  
        f_r_sonar = 1 
        r_sonar = 2
        b_r_sonar = 3
        b_sonar = 4
        b_l_sonar = 5
        l_sonar = 6
        f_l_sonar = 7
	# this is a count to ensure that the robot varies which direction it turns
	# when avoiding obstacles
	avoid_direction_count = 0
	#this indicates the direction the robot will turn when avoiding obstacles
	avoid_direction = 1 
	
	#these are true when there is a cliff in front or behind the robot
	tof_drive_ok_f = False 
	tof_drive_ok_b = False 
	
	# true when the setMotors function prevents the robot from moving due to immediate obstacles
	danger_prevented = False 

	# true when the bump sensor has detected that the robot has run into something
	b_bump = False
	f_bump = False

	# true when there is an immediate danger to the robot directly in front or behing
	# i.e. close object, cliff, bump. When true, the motors are prevented 
	# from driving in that direction. 
	immediate_danger_f = False #true when there is a close object or a cliff in front
	immediate_danger_b = False

	# False when the IMU is no longer level
	is_level = True 
	# True when the sensor_watch node thinks that all sensors are online and working
	sensor_ok = False 

	# for troubleshooting/monitoring  purposes, this is published to a topic 
	current_activity = 'starting up' 

	# the drive value that the motor is using (between -1 and 1
	current_l = 0 

	#holds the distances for each sonar
	sonar_dis = [0]*num_of_sonar 
	# holds the x- an y-coordinates of objects detected by each sonar
	x_cord = [0]*num_of_sonar 
	y_cord = [0]*num_of_sonar 
	avgCord()

	# true when the robot is turning to edgefind. Used to control how long the robot 
	# sleeps between chaning pwm values
	isAvoiding = False 

	GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pwm.set_pwm_freq(50)
	setMotors(0,0) #so that the robot isn't moving at startup
		
def accelerate(drive_target, turn_target): 
	#changes the speed of the motor with a fixed max rate (determined by accel_const)
	#only changes the forward/back velocity gradually - turn velocity is an instantaneous change
	# is the setMotors function prevent the robot from driving, this function stops trying to accelerate
	global current_l, danger_prevented 
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

def setMotors(joy_drive, joy_turn): 
	#sets the motors to a new PWM 
	global  current_l, current_activity, danger_prevented, terrible_mistake
	# these next lines prevent the robot from driving backward or forward when 
	# there is an immediate danger (cliff, close object, bump)
	# or when one or more sensors are offline
	findDangers()
	print "immediate dangers ", immediate_danger_f, immediate_danger_b
	if not sensor_ok:
		# the robot stops moving entirely
		joy_drive = 0
		joy_turn = 0
		danger_prevented = True
		terrible_mistake = False
		current_activity = "Waiting for all sensors to come back online"
	elif (joy_drive > 0) & (immediate_danger_f):
		# the robot stops suddenly and is prevented from driving forward 
		joy_drive = 0
		danger_prevented = True
		terrible_mistake = True
		current_activity = "Danger detected in front - prevented from moving forward"
	elif (joy_drive < 0) & (immediate_danger_b):
		# the robot stops suddenly and is prevented from driving backwards
		joy_drive = 0
		danger_prevented = True
		terrible_mistake = True
		current_activity = "Danger detected behind - prevented from driving back"
      	current_l = joy_drive
	new_v_drive = calcPWM(joy_drive)
	new_v_turn = calcPWM(joy_turn) 
        pwm.set_pwm(drivePin, 0, new_v_drive)
	pwm.set_pwm(turnPin, 0, new_v_turn)
	# sleeps for a set amount of time to prevent accelerating to quickly and jerking 
	if isAvoiding: 
		#because sleep_time_edgefinding is a smaller value than sleep_time
		#this helps prevent the robot from overshooting its turn
		time.sleep(sleep_time_edgefinding)
	else:
		time.sleep(sleep_time) 

def calcPWM(joy_value):
	#calculates the PWM value for a joystick value (between -1 and 1)
	#skips the range of zero PWM values unless joy_value is zero
	# since the motor drivers only take int PWM values, only returns int values
	# so the range of value is limited
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
	# "right direction" being that the edge is in the range determined by right_direction and other constants
	global isAvoiding, current_activity
	# this is so that the pause between setting pwm values is shorter to avoid overshooting 
	isAvoiding = True 
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

def reverse(go_in_direction): 
	# more of a reverse and turn
	# if the robot needs to back up and find a new course
	# go_in_direction is -1 if it needs to back up (obstacle in front)
	# or 1 if it needs to go straight (obstacle behind)
	global isAvoiding, current_activity, avoidance_count
	isAvoiding = True
	c_l = current_l
	if (c_l == 0) & ((not is_level) | f_bump | (not tof_drive_ok_f)):
		# this is to ensure that the robot always backs up if 
		# it's not level or if it hits something in front or there is a cliff
		c_l = 0.5
	elif (c_l ==0) & ( b_bump | (not tof_drive_ok_b)):
		# to ensure that the robot always drives forward if it 
		# hits something behind or if there is a cliff
		c_l = -0.5
	if avoidance_count >= 5:
		# if the robot has been trying to turn to avoid the same obstacles for a while
		# and has switched the directions it's turning to try and avoid the obstacle at 
		# at least five times it backs up instead
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
	# every once in a while, this switches the direction it turns when avoiding obstacles
	# this helps prevent the robot from getting suck
	# avoid_direction is +1 when it turns clockwise and -1 when it turns counterclockwise
	global avoid_direction, avoid_direction_count, avoidance_count
	if avoid_direction_count >= 15:
		avoid_direction = -avoid_direction
		avoid_direction_count = 0
		avoidance_count +=1
	else:
		avoid_direction_count +=1 

def findDangers():
	#determines if there are an immediate dangers
	# (cliff, object, the robot hit something) in front or behind
	# this is called every time the motors are set to a new PWM value
	# to do a last-minute check that there is nothing that the robot 
	# will hit/fall down

	global immediate_danger_f, immediate_danger_b
	if (not tof_drive_ok_f) | (sonar_dis[f_sonar] < front_distance ) | f_bump:
 		immediate_danger_f = True
	else:
		immediate_danger_f = False
        if (not tof_drive_ok_b) | (sonar_dis[b_sonar] < front_distance) | b_bump:  
                immediate_danger_b = True
        else:
                immediate_danger_b = False

def motion(): 
	#main driving function 
	global current_activity, avoidance_count, terrible_mistake
	current_activity = "calculating motion"
	avgCord()
	if not sensor_ok: 
		#one or more of the sensors are down, so it stops and waits for the sensors to come online again
		accelerate(0,0)
		terrible_mistake = False
		avoidance_count = 0 #resets this count since the robot did not have to reverse
		current_activity  = "waiting for all sensors to come online again"
	elif ((not tof_drive_ok_f) & (not tof_drive_ok_b)): 
		#stops and turns if there are cliffs in front and back
		setMotors(0,0)
		reverse(0)
		terrible_mistake = True
		current_activity = "cliff detected in front and behind - %f" % (avoid_direction*obstacle_avoidance_speed)
	elif ((not tof_drive_ok_f)| (f_bump)) | (not is_level):
		# stops, backs up, and turns if there is a cliff, it ran into something,
		# or is not level.
		setMotors(0,0)
		reverse(-1)
		terrible_mistake = False
		#sets current_activity to the correct activity
		if not tof_drive_ok_f:
			current_activity = "cliff detected in front  reversing %f" % (avoid_direction*obstacle_avoidance_speed)
		elif f_bump:
			terrible_mistake = True
			current_activity = "robot hit something in front, reversing %f" % (avoid_direction*obstacle_avoidance_speed)
		else:
			current_activity = "no longer level - reversing %f" % (avoid_direction*obstacle_avoidance_speed)
	elif ((not tof_drive_ok_b) | (b_bump)): 
		#if there is a cliff behind or it ran into something, it stops, goes forward, and turns
		setMotors(0,0)
		reverse(1)
		terrible_mistake = False
		if not tof_drive_ok_b:
			current_activity = "cliff detected behind, driving forward %f" % (avoid_direction*obstacle_avoidance_speed)
		else:
			terrible_mistake = True
			current_activity = "robot hit something in back, driving forward %f" % (avoid_direction*obstacle_avoidance_speed)
	elif (sonar_dis[f_sonar] < tooCloseDis)| (sonar_dis[f_l_sonar] < deadzone) | (sonar_dis[f_r_sonar] < deadzone):
		#when there is something in front of the robot it stops and turns
		current_activity = "obstacle - stopping and turning %f" % (avoid_direction*obstacle_avoidance_speed)
		reverse(-1*how_close)
		terrible_mistake = False
	elif (weighted_dis_avg > deadzone) & (weighted_dis_avg < edge_dis) & (how_close > close_quarters_edgefinding_lim):
		#so the robot only edgefinds when it is in a relatively open space  
		edgefindTurn() 
		terrible_mistake = False
                avoidance_count = 0 #resets the avoidance count since it didn't need to avoid anything
	else: 
		#when none of the sensors are reporting an immediate concern
		# and the robot is in too confined of a space to edgefind or there are no nearby edges
		current_activity = "driving"
		terrible_mistake = False
		if how_close <= close_quarters_edgefinding_lim:
			current_activity = "driving -- too close for edgefinding"
                avoidance_count = 0 #resetting this, since it didn't avoid anything
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
			weight = (abs(edge_dis - sonar_dis[i])) **2.5
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

        #determines how open the space it's in is 
        #i.e. middle of desk spaces or not
        # the smaller the value of how_close the more confined the area
	# how_close is compared to close_quarters_edgefinding_lim (a constant)
	# to determine if the space is too cramped to edgefind
        global how_close
        how_close = 0
        for i in range(len(sonar_dis)):
                if (sonar_dis[i] <= edge_dis) & (sonar_dis[i] > deadzone):
                        how_close += (sonar_dis[i]/edge_dis)**10
                elif (sonar_dis[i] > edge_dis):
                        how_close +=1
        how_close = how_close /len(sonar_dis)

def sonarlistener(data): 
	#listens for the sonar data
	global sonar_dis
	sonar_dis = data.distances

def imuListener(data): 
	#the imu checks whether the robot is level
	global is_level
	is_level = data.is_level   	

def sensorStatusListener(data): 
	#this node checks whether the sensors are online and reports True if all sensors are working
	global sensor_ok
	sensor_ok = data.sensor_system_status
	
def tofListener(data): 
	# the time of flight sensor checks to see if there is 
	# a cliff/sudden drop of floor in front or behind the robot
	global tof_drive_ok_f, tof_drive_ok_b
	tof_drive_ok_f = data.drive_front_ok
	tof_drive_ok_b = data.drive_back_ok

def bumpSkirtListener(data):
	# this checks whether the robot has bumped into something
	global f_bump, b_bump
	f_bump = data.f_bump
	b_bump = data.b_bump
	
def listener():
	rospy.Subscriber("remyjr/imu_data", imu_data, imuListener, queue_size = 1)
	rospy.Subscriber("remyjr/sonar_data", sonar_array, sonarlistener, queue_size = 1)
	rospy.Subscriber("remyjr/sensor_status", sensor_status, sensorStatusListener, queue_size = 1)	
	rospy.Subscriber("remyjr/tof_data", tof_data, tofListener, queue_size=1)
	rospy.Subscriber("remyjr/bump_data", bump_data, bumpSkirtListener, queue_size=1)
	
def publishData(): 
	# sends the data to the graphing node and current_activity node
	global pub1, pub2, pub3
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
	motion_data.close_quarters_edgefinding_lim = close_quarters_edgefinding_lim
	motion_data.close_quarters = how_close
	pub1.publish(motion_data)
	activity = String()
	activity.data = current_activity
	mistake = Bool()
	mistake.data = terrible_mistake
	pub2.publish(activity)
	pub3.publish(mistake)

def mainLoop():
	rospy.init_node('robot_controller')
	global pub1, pub2, pub3 
	pub1=rospy.Publisher('remyjr/motion_data', remyjr_graphs, queue_size=1) 
	pub2=rospy.Publisher('remyjr/current_activity', String, queue_size=1)
	pub3=rospy.Publisher('remyjr/oops', Bool, queue_size=1)
	while not rospy.is_shutdown():
	   try:
		listener() #listens to sonar data
		motion() #moves
		publishData() #sends data to graphing node
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



