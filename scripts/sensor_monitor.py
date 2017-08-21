#!/usr/bin/env python

import rospy
from remyJr.msg import tof_data
from remyJr.msg import imu_data
from remyJr.msg import sensor_status
from remyJr.msg import sonar_array
from remyJr.msg import bump_data
from remyJr.msg import current_activity
from remyJr.msg import motor_data


monitor_rate = 4 # in Hz. This need to be slower then any of the sensor nodes rates

# these functions report whether the specific sensor is online
# since each message published by a sensor is tagged with a publish count,
# it can check to see if the current_sensor_time (current publish count)
# is the same as the previous one. If it is, it know that the sensor hasn't
# updated since last time it went through this function, so it reports that that sensor is offline
# this relies on the sensors publishing faster then this sensor monitor loops
def reportSonarStatus():
	global sonar_ok, last_sonar_time
	if last_sonar_time != current_sonar_time:
		sonar_ok = True
	else:
		sonar_ok = False
	last_sonar_time = current_sonar_time

def reportIMUStatus():
        global imu_ok, last_imu_time
        if last_imu_time != current_imu_time:
                imu_ok = True
        else:
                imu_ok = False
        last_imu_time = current_imu_time

def reportTOFStatus():
        global tof_ok, last_tof_time
        if last_tof_time != current_tof_time:
                tof_ok = True
        else:
                tof_ok = False
        last_tof_time = current_tof_time

def reportMotorControlStatus():
        global motor_control_ok, last_motor_control_time
        if last_motor_control_time != current_motor_control_time:
                motor_control_ok = True
        else:
                motor_control_ok = False
        last_motor_control_time = current_motor_control_time

def reportBrainStatus():
        global brain_ok, last_brain_time
        if last_brain_time != current_brain_time:
                brain_ok = True
        else:
                brain_ok = False
        last_brain_time = current_brain_time

def reportBumpSkirtStatus():
        global bump_skirt_ok, last_bump_skirt_time
        if last_bump_skirt_time != current_bump_skirt_time:
                bump_skirt_ok = True
        else:
                bump_skirt_ok = False
        last_bump_skirt_time = current_bump_skirt_time

# these are all listeneing to a specific sensor's data topic to update
# the current_sensor_time (current publish count)
def getSonarTime(data):
	global current_sonar_time
	current_sonar_time = data.publish_count

def getIMUTime(data):
	global current_imu_time
	current_imu_time = data.publish_count

def getTOFTime(data):
        global current_tof_time
        current_tof_time = data.publish_count

def getMotorControlTime(data):
        global current_motor_control_time
        current_motor_control_time = data.publish_count

def getBrainTime(data):
        global current_brain_time
        current_brain_time = data.publish_count

def getBumpSkirtTime(data):
	global current_bump_skirt_time
	current_bump_skirt_time = data.publish_count

def statusListener():
	rospy.Subscriber("remyjr/motor_data", motor_data, getMotorControlTime)
	rospy.Subscriber("remyjr/current_activity", current_activity, getBrainTime)
	rospy.Subscriber("remyjr/sonar_data", sonar_array, getSonarTime)
	rospy.Subscriber("remyjr/imu_data", imu_data, getIMUTime)
	rospy.Subscriber("remyjr/tof_data", tof_data, getTOFTime)
	rospy.Subscriber("remyjr/bump_data", bump_data, getBumpSkirtTime)

def findSystemStatus(): #checks the 'heartbeat' of the sensors and returns true only if all sensors are working
	systems_go = True
	reportSonarStatus()
	reportIMUStatus()
	reportTOFStatus()
	reportBumpSkirtStatus()
	reportBrainStatus()
	reportMotorControlStatus()
	if imu_ok & sonar_ok & tof_ok &bump_skirt_ok & motor_control_ok:
		systems_go = True
	else:
		systems_go = False
	return systems_go

def reportStatus():
	global last_sonar_time, last_imu_time, current_imu_time, current_sonar_time, last_tof_time, current_tof_time
	global last_bump_skirt_time, current_bump_skirt_time, last_motor_control_time, current_motor_control_time
	global last_brain_time, current_brain_time
	last_sonar_time = 20001
	last_imu_time = 200001
	last_tof_time = 300002
	last_bump_skirt_time = 3000002
	last_brain_time = 34000003
	last_motor_control_time = 300002
	current_motor_control_time = 30000021
	current_brain_time = 3000002
	current_bump_skirt_time = 200033
	current_tof_time = 20000003
	current_imu_time = 2000003
	current_sonar_time = 20000002
	pub = rospy.Publisher('remyjr/sensor_status', sensor_status, queue_size = 1)
	rospy.init_node('sensor_watch', anonymous=True)
	rate = rospy.Rate(monitor_rate)
	publish_count = 0
	while not rospy.is_shutdown():
		statusListener()
		systems_go = sensor_status()
		systems_go.publish_count = publish_count
                systems_go.sensor_system_status = findSystemStatus()
		systems_go.imu_status = imu_ok
		systems_go.sonar_status = sonar_ok
		systems_go.tof_status = tof_ok
		systems_go.brain_status = brain_ok
		systems_go.bump_skirt_status = bump_skirt_ok
		systems_go.motor_control_status = motor_control_ok
		rospy.loginfo(systems_go)
		pub.publish(systems_go)
		publish_count += 1
		if publish_count >= 2000:
			publish_count = 0
		rate.sleep()

if __name__ == '__main__':
	try:
		reportStatus()
	except rospy.ROSInterruptException:
		pass
