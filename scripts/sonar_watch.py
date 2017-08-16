#!/usr/bin/env python
import rospy
import time
import numpy as np
from sensor_msgs.msg import Joy
from remyJr.msg import sonar_array
import RPi.GPIO as GPIO
import Adafruit_MCP3008
from std_msgs.msg import Int32

#when testing specific sonar or drive features, this allows false data (or edge-case data) to be passed forward
testing = False
test_sonar_num = 3 #the specific sonar that's beign tested
test_dis = 150 #the false distance given to all the other sonars

# Software SPI configuration:
CLK  = 18
MISO = 27
MOSI = 17
CS   = 4
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#ADC channel numbers for each sensor
f_sonar = 3
f_r_sonar = 0
r_sonar = 2
b_r_sonar = 5
b_sonar = 1
b_l_sonar = 7
l_sonar = 6
f_l_sonar = 4

#determines the order that the distances are passed to the topic
#currently clockwise from front
pin_array = [f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar]

# this number represents the number of times a sonar has to return the same-ish number before
# this node will consider it overheated
overheat_threshold = 1000

sonar_rate = 30 #in hz, this is how quickly the program runs

sonar_array_trig_pin = 5
num_of_sonar = 8
#this is the number of past readings of sonar the node remembers
queue_len = 7

def resetArray():
	GPIO.setup(sonar_array_trig_pin, GPIO.OUT)
	        #triggers the first sonar
        GPIO.output(sonar_array_trig_pin, GPIO.LOW)
	time.sleep(0.13) #gives the array time to stop ranging
        GPIO.output(sonar_array_trig_pin, GPIO.HIGH) #starts the array ranging
        time.sleep(.001)
        GPIO.output(sonar_array_trig_pin, GPIO.LOW)

        time.sleep(0.003)
        GPIO.setup(sonar_array_trig_pin, GPIO.IN) #allows the sonar array to trigger itself

def moveQueue():
	global dis_queue
	for i in range(num_of_sonar):
		for j in range(queue_len -1):
			dis_queue[i,j] = dis_queue[i, j+1]
		dis_queue[i, queue_len -1] = 0

def avgQueue(i):
	sum = 0
	for j in range(queue_len):
		sum = sum + dis_queue[i,j]
	avg =int( sum / queue_len)
	return avg

def getMedian():
	#using medians instead of averages means that the robot responds slower
	global medians
	medians = np.zeros(num_of_sonar)
	temp = np.sort(dis_queue)
	print temp
	for i in range(num_of_sonar):
		medians[i] = temp[i, int(queue_len/2)]
	return medians

def readSonarArray():
	#reads sonars
	#the final sonar values it gives, sonar_dist, is an average of the
	# last queue_len num of values. This is to help prevent noise (big jumps
	# where a sonar suddenly sees something thats not there)
        global sonar_dist, dis_queue, c_dist, something_overheated, overheat_count
	c_dist = [0]*num_of_sonar
        moveQueue()
	something_overheated = False
        for i in range(num_of_sonar):
	        dis_queue[i,queue_len-1] = mcp.read_adc(i)
		if (dis_queue[i, queue_len -1] > 225) & (dis_queue[i, queue_len -1] < 260):
			overheat_count[i] +=1
			# overheated sonars usually give distance readings between 225 and 255
		else:
			overheat_count[i] =0
		if overheat_count[i] >=overheat_threshold:
			# since overheated sonars read same-ish values, when a sonar has been reading
			# an overheated value for a long time it might be overheated
			something_overheated = True
		c_dist[i] = dis_queue[i, queue_len-1]
		sonar_dist[i] = avgQueue(i)
	orderSonarDis()

def orderSonarDis():
	#orders the distances of the sonars so that the entires
	#correspond to each sensor in clockwise order starting with the front sensor
	#make sure that the order of temp is the same as that of pin array
	global sonar_dist
	temp = [0]*num_of_sonar
	for i in range(num_of_sonar):
		temp[i] = sonar_dist[pin_array[i]]
	sonar_dist = temp

def start():
	global dis_queue, avg_dis, sonar_dist, overheat_count, something_overheated
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(sonar_array_trig_pin, GPIO.OUT)
	dis_queue = np.zeros((num_of_sonar, queue_len))
	sonar_dist = np.zeros(num_of_sonar)
	overheat_count = np.zeros(num_of_sonar)
	something_overheated = False

	#triggers the first sonar
	GPIO.output(sonar_array_trig_pin, GPIO.LOW)
        GPIO.output(sonar_array_trig_pin, GPIO.HIGH)
        time.sleep(.001)
        GPIO.output(sonar_array_trig_pin, GPIO.LOW)

	#allows the last sonar to trigger the first again, to allow continuous ranging
	#the sonar trig pin must be set to an input, or the raspberry pi will hold the last value
        time.sleep(0.003)
	GPIO.setup(sonar_array_trig_pin, GPIO.IN)
	pub = rospy.Publisher('remyjr/sonar_data', sonar_array, queue_size=1)
	rospy.init_node('sonarreader')
	rate = rospy.Rate(sonar_rate)
	publish_count = 0
	reset_count = 0
	overheated  = False
	while not rospy.is_shutdown():
	        sonar = sonar_array()
		readSonarArray()
		sonar.publish_count = publish_count
		sonar.distances = sonar_dist
		if testing == True: #allows false or edge-case data to be given if something is beign tested
			sonar.distances = [150]*num_of_sonar
			sonar.distances[test_sonar_num] = test_dis
		rospy.loginfo(sonar)
	        pub.publish(sonar)
		if not something_overheated:
			publish_count +=1
		else:
			rospy.loginfo(overheat_count)
		reset_count +=1
		if publish_count>=2000:
			publish_count = 0
		if reset_count >=100:
			resetArray()
			reset_count = 0
		rate.sleep()
	#setting the sonar trig pin to a low output stops the array from continuously ranging
	#by preventing the last sonar from triggering the first
 	GPIO.setup(sonar_array_trig_pin, GPIO.OUT)
	GPIO.setup(sonar_array_trig_pin, GPIO.LOW)


if __name__ == '__main__':
	try:
		start()
	except KeyboardInterrupt:
		pass
