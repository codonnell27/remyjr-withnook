#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt


plt.ion()
n = 0
while not rospy.is_shutdown():
	plt.clf()
	plt.plot([1,2,3,4, n])

	plt.draw()
	plt.pause(0.001)
	print n
	n +=1
