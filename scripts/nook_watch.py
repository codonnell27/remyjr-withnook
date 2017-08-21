#!/usr/bin/env python
import rospy
from remyJr.msg import sensor_status
from remyJr.msg import nook_status

monitor_rate = 2 # in Hz, this needs to be the same or slower then the sensor_monitor rate


# this node reports whether the nook is commected to the raspberry pi
# since each message published by the sensor monitor node is tagged with a publish count,
# it can check to see if the current_sensor_time (current publish count)
# is the same as the previous one. If it is, it know that the nook hasn't
# updated since last time it went through this function, so it reports that
# the nook isn't working
def reportNookStatus():
        global nook_ok, last_nook_time
        if last_nook_time != current_nook_time:
                nook_ok = True
        else:
                nook_ok = False
        last_nook_time = current_nook_time

# these are all listeneing to a specific sensor's data topic to update
# the current_sensor_time (current publish count)
def getNookTime(data):
        global current_nook_time
        current_nook_time = data.publish_count

def statusListener():
        rospy.Subscriber("remyjr/sensor_status", sensor_status, getNookTime)

def findSystemStatus(): #checks the 'heartbeat' of the nook and returns true only if the nook is working
        systems_go = True
        reportNookStatus()
        if nook_ok:
                systems_go = True
        else:
                systems_go = False
        return systems_go

def reportStatus():
        global last_nook_time, current_nook_time
        last_nook_time = 20001
        current_nook_time = 30000021
        pub = rospy.Publisher('remyjr/nook_status', nook_status, queue_size = 1)
        rospy.init_node('nook_watch', anonymous=True)
        rate = rospy.Rate(monitor_rate)
        publish_count = 0
        while not rospy.is_shutdown():
                statusListener()
                systems_go = nook_status()
                systems_go.publish_count = publish_count
                systems_go.nook_status = findSystemStatus()
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

