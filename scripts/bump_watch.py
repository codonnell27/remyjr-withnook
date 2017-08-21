#!/usr/bin/env python
import rospy
import time
from remyJr.msg import bump_data
import RPi.GPIO as GPIO
import Adafruit_MCP3008

# System Dependent Variables

# Software SPI configuration:
CLK  = 22 #CLK pin on ADC
MISO = 23 #Dout pin on ADC
MOSI = 24 #Din pin on ADC
CS   = 25 #CS/SHDN pin on ADC
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#ADC channel numbers for each sensor
x_sensor =7
y_sensor =5
btn_sensor =6

# System Independent Variabes (assuming the same bump skirt is used)

# these is a pretty wide threshold to help minimize false positives
# anytime the bump sensor doesn't equalize after a bump
bump_threshold_uplim = 550
bump_threshold_lowlim = 490

def readJoystick():
	# reads the joystick and decides whether the robot has hit something
	# f_bump is true when the robot has hit something in front, etc.
	# requires two consecutive readings of a bump before it reports that
	# the robot has hit something. This is to prevent false positives
        global f_bump, b_bump, r_bump, l_bump, last_f, last_b, last_r, last_l

	x_axis_val = mcp.read_adc(x_sensor)
	y_axis_val = mcp.read_adc(y_sensor)

	print "x-axis data " , 	x_axis_val, "y-axis data ", y_axis_val

	if x_axis_val <=bump_threshold_lowlim:
		if last_f:
			f_bump = True
		else:
			f_bump = False
		last_f = True
	else:
		f_bump = False
		last_f = False

        if x_axis_val >=bump_threshold_uplim:
		if last_b:
	                b_bump = True
		else:
			b_bump = False
		last_b = True
        else:
                b_bump = False
		last_b = False
        if y_axis_val <=bump_threshold_lowlim:
		if last_r:
	                r_bump = True
		else:
			r_bump = False
		last_r = True
        else:
		last_r = False
                r_bump = False
        if y_axis_val >=bump_threshold_uplim:
		if last_l:
	                l_bump = True
		else:
			l_bump = False
		last_l = True
        else:
		last_l = False
                l_bump = False

def start():
	global last_l, last_r, last_f, last_b
	last_l = False
	last_r = False
	last_f = False
	last_b = False
        GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	pub = rospy.Publisher('remyjr/bump_data', bump_data, queue_size=1)
	rospy.init_node('bumpskirt')
	rate = rospy.Rate(20)
	publish_count = 0
	while not rospy.is_shutdown():
	        data = bump_data()
		readJoystick()
		data.publish_count = publish_count
		data.f_bump = f_bump
		data.b_bump = b_bump
		data.r_bump = r_bump
		data.l_bump = l_bump
		print data
		rospy.loginfo(data)
	        pub.publish(data)
		publish_count +=1
		if publish_count>=2000:
			publish_count = 0
		rate.sleep()

if __name__ == '__main__':
	try:
		start()
	except KeyboardInterrupt:
		pass
