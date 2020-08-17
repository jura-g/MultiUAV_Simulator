#!/usr/bin/env python
import roslib; roslib.load_manifest('graupner_serial')
import rospy

import math, numpy
#from math import acos
#from math import hypoth

from graupner_serial.msg import RCchannelData

from nav_msgs.msg import Odometry

from graupner_serial.msg import BatteryStatus


#from mav_msgs import msgMultiDofJointTrajectoryFromPositionYaw

import uinput, time
import pygame, sys, os
from pygame.locals import *

import threading

selectedUAV = 0
number_of_UAVs = 3

X_cur_position = [0 for x in range(number_of_UAVs+1)]
Y_cur_position = [0 for x in range(number_of_UAVs+1)]
Z_cur_position = [0 for x in range(number_of_UAVs+1)]
W_cur_orientation = [0 for x in range(number_of_UAVs+1)]
cur_odometry_time = [0.0 for x in range(number_of_UAVs+1)]

X_old_position = [0 for x in range(number_of_UAVs+1)]
Y_old_position = [0 for x in range(number_of_UAVs+1)]
Z_old_position = [0 for x in range(number_of_UAVs+1)]
W_old_orientation = [0 for x in range(number_of_UAVs+1)]
old_odometry_time = [0.0 for x in range(number_of_UAVs+1)]

h_speed = [0 for x in range(number_of_UAVs+1)]
v_speed = [0 for x in range(number_of_UAVs+1)]

first_odometry_callback = [True for x in range(number_of_UAVs+1)]

MAX_UAV_battery = [100 for x in range(number_of_UAVs+1)]
UAV_battery = [100 for x in range(number_of_UAVs+1)]

#/* ODOMETRY handlers */
def OdometryCallback(data, UAV_ID):

	global first_odometry_callback

	if first_odometry_callback[UAV_ID]:
		old_odometry_time[UAV_ID] = data.header.stamp.secs + float(data.header.stamp.nsecs)/1000000000
		first_odometry_callback[UAV_ID] = False
		

	if (cur_odometry_time[UAV_ID] - old_odometry_time[UAV_ID]) > 1:

 		h_speed[UAV_ID] = math.hypot((X_cur_position[UAV_ID]-X_old_position[UAV_ID]),(Y_cur_position[UAV_ID]-Y_old_position[UAV_ID]))/(cur_odometry_time[UAV_ID]-old_odometry_time[UAV_ID])
		v_speed[UAV_ID] = (Z_cur_position[UAV_ID]-Z_old_position[UAV_ID])/(cur_odometry_time[UAV_ID]-old_odometry_time[UAV_ID])

		X_old_position[UAV_ID] = X_cur_position[UAV_ID]
		Y_old_position[UAV_ID] = Y_cur_position[UAV_ID]
		Z_old_position[UAV_ID] = Z_cur_position[UAV_ID]
		W_old_orientation[UAV_ID] = W_cur_orientation[UAV_ID]
		old_odometry_time[UAV_ID] = cur_odometry_time[UAV_ID]
		
		if (Z_cur_position[UAV_ID] > 0.1):
			if ((h_speed[UAV_ID] > 0.1) or (v_speed[UAV_ID] > 0.1)):
 				UAV_battery[UAV_ID] = UAV_battery[UAV_ID] - 2
			else:
				UAV_battery[UAV_ID] = UAV_battery[UAV_ID] - 1

	X_cur_position[UAV_ID] = data.pose.pose.position.x
	Y_cur_position[UAV_ID] = data.pose.pose.position.y
	Z_cur_position[UAV_ID] = data.pose.pose.position.z
	W_cur_orientation[UAV_ID] = 2 * math.acos(data.pose.pose.orientation.w)
	cur_odometry_time[UAV_ID] = data.header.stamp.secs + float(data.header.stamp.nsecs)/1000000000

def baterry_init():
	global UAV_name	
	
	UAV_name = rospy.get_param('uav_name_param')
	rospy.Subscriber(UAV_name+"_sim1/odometry_sensor1/odometry", Odometry, OdometryCallback, callback_args=1)
	rospy.Subscriber(UAV_name+"_sim2/odometry_sensor1/odometry", Odometry, OdometryCallback, callback_args=2)
	rospy.Subscriber(UAV_name+"_sim3/odometry_sensor1/odometry", Odometry, OdometryCallback, callback_args=3)

	MAX_UAV_battery[1] = 100
	MAX_UAV_battery[2] = 1000
	MAX_UAV_battery[3] = 100

	UAV_battery[1] = MAX_UAV_battery[1]
	UAV_battery[2] = MAX_UAV_battery[2]
	UAV_battery[3] = MAX_UAV_battery[3]


def send_batt_data():

	UAV_battery_pub = rospy.Publisher('BatteryStatusforGUI', BatteryStatus, queue_size=10)
	msgUAVBatt = BatteryStatus()
	for n in range(1, number_of_UAVs+1):
		msgUAVBatt.maxBatt[n] = MAX_UAV_battery[n]
		msgUAVBatt.curBatt[n] = UAV_battery[n]
	UAV_battery_pub.publish(msgUAVBatt)
		
def main():

	rospy.init_node('BatteryNode')

	baterry_init()

	rate = rospy.Rate(1000)
	while not rospy.is_shutdown():
		send_batt_data()
		rate.sleep()

if __name__ == "__main__":
	main()
