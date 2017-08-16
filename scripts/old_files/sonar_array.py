#!/usr/bin/env python
import time
import rospy
import Adafruit_MCP3008
#import sonar_array.msg
import RPi.GPIO as GPIO

# Software SPI configuration:
CLK  = 22
MISO = 23
MOSI = 24
CS   = 25
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#pin numbers on the ADC for the various sonars
front_sonar = 1
front_left_sonar = 2 
front_right_sonar = 0
left_sonar = 3
right_sonar = 6
back_sonar = 5
back_right_sonar = 7 
back_left_sonar = 4

sonar_array_trig_pin = 27
sonar_array_read_complete_pin = 21


num_of_sensors = 8
beam_width = 45 #degrees, an estimate
def setup():
	print ('Setting up...')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
	GPIO.setup(sonar_array_trig_pin, GPIO.OUT)
	GPIO.setup(sonar_array_read_complete_pin, GPIO.IN)
        GPIO.output(sonar_array_trig_pin, GPIO.HIGH)
        time.sleep(.001)
        GPIO.output(sonar_array_trig_pin, GPIO.LOW)
        time.sleep(0.11*num_of_sensors)
	print ('Setup complete!')

def loop():
    print('Reading MCP3008 values, press Ctrl-C to quit...')

    while True:
	# Read all the ADC channel values in a list.
	values = [0]*8
	print ("Triggering array...")
	GPIO.output(sonar_array_trig_pin, GPIO.HIGH)
	time.sleep(.001)
	GPIO.output(sonar_array_trig_pin, GPIO.LOW)
	time.sleep(0.05*num_of_sensors)
	print ('Reading array...')
#	for i in range(10):
    	for i in range(num_of_sensors):
	        # The read_adc function will get the value of the specified channel (0-7).
       		values[i] = mcp.read_adc(i)
	print values[front_sonar], values[front_right_sonar], values[right_sonar], values[back_right_sonar], values[back_sonar], values[back_left_sonar], values[left_sonar], values[front_left_sonar]
#	print values[front_sonar] 
	#time.sleep(0.5)

def destroy():
	GPIO.cleanup()

if __name__ == '__main__':
	setup()
	try:
		loop()
	except KeyboardInterrupt:
		destroy()
