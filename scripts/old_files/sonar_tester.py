#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy
from remyJr.msg import sonar_array
import RPi.GPIO as GPIO
import Adafruit_MCP3008
from std_msgs.msg import Int32

testing = False
test_sonar_num = 3
test_dis = 100

# Software SPI configuration:
CLK  = 22
MISO = 23
MOSI = 24
CS   = 25
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#ADC channel numbers for each sensor
f_sonar = 0
f_r_sonar = 6
r_sonar = 7
b_r_sonar = 5
b_sonar = 4
b_l_sonar = 3
l_sonar = 2
f_l_sonar = 1

#determines the order that the distances are passed to the topic
#currently clockwise from front
pin_array = [f_sonar, f_r_sonar, r_sonar, b_r_sonar, b_sonar, b_l_sonar, l_sonar, f_l_sonar]

sonar_array_trig_pin = 27
num_of_sonar = 8

def readSonarArray(): #reads sonar
        global sonar_dist
        sonar_dist = [0]*num_of_sonar

        for i in range(num_of_sonar):
	        sonar_dist[i] = mcp.read_adc(i)
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

def getCurrentTime():
	global current_time
	current_time = time.time()	

def start():
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(sonar_array_trig_pin, GPIO.OUT)

	#triggers the first sonar
	GPIO.output(sonar_array_trig_pin, GPIO.LOW)
        GPIO.output(sonar_array_trig_pin, GPIO.HIGH)
        time.sleep(.001)
        GPIO.output(sonar_array_trig_pin, GPIO.LOW)

	#allows the last sonar to trigger the first again, to allow continuous ranging
	#the sonar trig pin must be set to an input, or the raspberry pi will hold the last value
        time.sleep(0.2)
	GPIO.setup(sonar_array_trig_pin, GPIO.IN)
	pub1 = rospy.Publisher('remyjr/sonar_status', Int32, queue_size=1)
	pub2 = rospy.Publisher('remyjr/sonar_dist', sonar_array, queue_size=1)
	rospy.init_node('sonarreader')	
	rate = rospy.Rate(20)
	publish_count = 0
	while not rospy.is_shutdown():
	        sonar = sonar_array()
		readSonarArray()
		sonar.pins = pin_array
		sonar.distances = sonar_dist	
		if testing == True:
			sonar.distances = [73]*num_of_sonar
			sonar.distances[test_sonar_num] = test_dis
		pub1.publish(publish_count)	
	        pub2.publish(sonar)
		publish_count +=1
		if publish_count >=2000:
			publish_count = 0
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
