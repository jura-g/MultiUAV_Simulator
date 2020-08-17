#!/usr/bin/env python
import roslib; roslib.load_manifest('graupner_serial')
import rospy

import math, numpy
#from math import acos
#from math import hypoth

from graupner_serial.msg import RCchannelData

from nav_msgs.msg import Odometry

from graupner_serial.msg import DesiredTrajectory, MissionWP, MissionStartStop

from graupner_serial.srv import GUITrajectoryParameters

#from mav_msgs import msgMultiDofJointTrajectoryFromPositionYaw

import uinput, time
import pygame, sys, os
from pygame.locals import *

import threading

class MissionControl:

    def __init__(self):
	self.selectedUAV = 0
	self.number_of_UAVs = 3
	self.MAX_WPs = 1000

	self.mission_started = False

	self.number_of_WPs = [0 for x in range(self.number_of_UAVs+1)]
	self.mission_active_flag = [0 for x in range(self.number_of_UAVs+1)]
	self.next_WP = [0 for x in range(self.MAX_WPs)]

	self.wpNumber = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)]
	self.WP_X = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)] 
	self.WP_Y = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)] 
	self.WP_Z = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)]
	self.WP_W = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)] 
	self.WP_LT = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)]

	self.Loiter = [False for x in range(self.number_of_UAVs+1)]
	self.Flight = [False for x in range(self.number_of_UAVs+1)]

	self.start_LT = [0 for x in range(self.number_of_UAVs+1)]

	self.X_cur_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
	self.Y_cur_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
	self.Z_cur_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
	self.W_cur_orientation = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]

	self.UAV_mode = [[1 for x in range(self.number_of_UAVs+1)] for y in range(2)]

	self.desiredX = [0.0 for x in range(90)]
	self.desiredY = [0.0 for x in range(90)]
	self.desiredZ = [0.0 for x in range(90)]
	self.desiredW = [0.0 for x in range(90)]

	self.UAV_name = rospy.get_param('uav_name_param')
	rospy.Subscriber(self.UAV_name+"_sim1/odometry_sensor1/odometry", Odometry, self.OdometryCallback, callback_args=1)
	rospy.Subscriber(self.UAV_name+"_sim2/odometry_sensor1/odometry", Odometry, self.OdometryCallback, callback_args=2)
	rospy.Subscriber(self.UAV_name+"_sim3/odometry_sensor1/odometry", Odometry, self.OdometryCallback, callback_args=3)

	rospy.Subscriber("MissionfromGUI", MissionWP, self.MissionWPCallback)
	rospy.Subscriber("MissionStartStop", MissionStartStop, self.StartStopCallback)

	self.request_TrajectoryFromGUIService = rospy.ServiceProxy('TrajectoryFromGUIService', GUITrajectoryParameters)

	print("MCN initialization")


    def run(self):
	rate = rospy.Rate(1000)
	while not rospy.is_shutdown():
	    for i in range(1, self.number_of_UAVs+1):
		if self.mission_active_flag[i]:
	  	    self.send_WP_data(i)
				
	    rate.sleep()

#/* ODOMETRY handlers */

    def OdometryCallback(self, data, UAV_ID):
	self.X_cur_position[0][UAV_ID] = data.pose.pose.position.x
	self.Y_cur_position[0][UAV_ID] = data.pose.pose.position.y
	self.Z_cur_position[0][UAV_ID] = data.pose.pose.position.z
	self.W_cur_orientation[0][UAV_ID] = math.atan2(2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y), 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z))


#/* MISSION handlers */

    def MissionWPCallback(self, data):
	self.selectedUAV = data.selectedUAV
	self.number_of_WPs[self.selectedUAV] = data.numberofWPs

	for i in range(self.number_of_WPs[self.selectedUAV]):
		self.wpNumber[self.selectedUAV][i] = data.wpNumber[i]
		self.WP_X[self.selectedUAV][i] = data.wpX[i]
		self.WP_Y[self.selectedUAV][i] = data.wpY[i]
		self.WP_Z[self.selectedUAV][i] = data.wpZ[i]
		self.WP_W[self.selectedUAV][i] = data.wpW[i]
		self.WP_LT[self.selectedUAV][i] = data.wpLT[i]


    def StartStopCallback(self, data):	
	self.selectedUAV = data.selectedUAV
	self.mission_active_flag[self.selectedUAV] = data.isActiveFlag
	self.next_WP[self.selectedUAV] = 0
	self.mission_started = False


    def send_WP_data(self, UAV_ID):

	if (self.next_WP[UAV_ID] == 0) and (not self.mission_started):
		
		#desired_tra_pub = rospy.Publisher(self.UAV_name+'_sim'+str(UAV_ID)+'/DesiredTrajectoryfromGUI', DesiredTrajectory, queue_size=10)

		#msgDesTra = DesiredTrajectory()
		#msgDesTra.desiredX = self.WP_X[UAV_ID][0]
		#msgDesTra.desiredY = self.WP_Y[UAV_ID][0]
		#msgDesTra.desiredZ = self.WP_Z[UAV_ID][0]
		#msgDesTra.desiredW = self.WP_W[UAV_ID][0]
		#desired_tra_pub.publish(msgDesTra)

		self.UAV_mode[0][UAV_ID] = 1

		self.desiredX[0] = self.X_cur_position[0][UAV_ID]
		self.desiredY[0] = self.Y_cur_position[0][UAV_ID]
		self.desiredZ[0] = self.Z_cur_position[0][UAV_ID]
		self.desiredW[0] = self.W_cur_orientation[0][UAV_ID]
		self.desiredX[1] = self.WP_X[UAV_ID][0]
		self.desiredY[1] = self.WP_Y[UAV_ID][0]
		self.desiredZ[1] = self.WP_Z[UAV_ID][0]
		self.desiredW[1] = self.WP_W[UAV_ID][0]

		resp = self.request_TrajectoryFromGUIService(UAV_ID, self.UAV_mode[0][UAV_ID], self.desiredX, self.desiredY, self.desiredZ, self.desiredW)
	
		self.Flight[UAV_ID] = True
		self.mission_started = True
		print("MCN test 1")
	

	if ((self.Flight[UAV_ID]) and (math.hypot((self.WP_X[UAV_ID][self.next_WP[UAV_ID]]-self.X_cur_position[0][UAV_ID]),(self.WP_Y[UAV_ID][self.next_WP[UAV_ID]]-self.Y_cur_position[0][UAV_ID])) < 0.2)):
		self.start_LT[UAV_ID] = rospy.get_time()
		self.Loiter[UAV_ID] = True
		self.Flight[UAV_ID] = False
		print("MCN test 2")


	if ((self.Loiter[UAV_ID]) and (rospy.get_time() - self.start_LT[UAV_ID] > self.WP_LT[UAV_ID][self.next_WP[UAV_ID]]) and (self.next_WP[UAV_ID] + 2 <= self.number_of_WPs[UAV_ID])): 
		self.Loiter[UAV_ID] = False
		self.Flight[UAV_ID] = True
		self.next_WP[UAV_ID] = self.next_WP[UAV_ID] + 1
		print("MCN test 3")

		#desired_tra_pub = rospy.Publisher(self.UAV_name+'_sim'+str(UAV_ID)+'/DesiredTrajectoryfromGUI', DesiredTrajectory, queue_size=10)

		#msgDesTra = DesiredTrajectory()
		#msgDesTra.desiredX = self.WP_X[UAV_ID][self.next_WP[UAV_ID]]
		#msgDesTra.desiredY = self.WP_Y[UAV_ID][self.next_WP[UAV_ID]]
		#msgDesTra.desiredZ = self.WP_Z[UAV_ID][self.next_WP[UAV_ID]]
		#msgDesTra.desiredW = self.WP_W[UAV_ID][self.next_WP[UAV_ID]]
		#desired_tra_pub.publish(msgDesTra)

		self.UAV_mode[0][UAV_ID] = 1

		self.desiredX[0] = self.X_cur_position[0][UAV_ID]
		self.desiredY[0] = self.Y_cur_position[0][UAV_ID]
		self.desiredZ[0] = self.Z_cur_position[0][UAV_ID]
		self.desiredW[0] = self.W_cur_orientation[0][UAV_ID]
		self.desiredX[1] = self.WP_X[UAV_ID][self.next_WP[UAV_ID]]
		self.desiredY[1] = self.WP_Y[UAV_ID][self.next_WP[UAV_ID]]
		self.desiredZ[1] = self.WP_Z[UAV_ID][self.next_WP[UAV_ID]]
		self.desiredW[1] = self.WP_W[UAV_ID][self.next_WP[UAV_ID]]

		resp = self.request_TrajectoryFromGUIService(UAV_ID, self.UAV_mode[0][UAV_ID], self.desiredX, self.desiredY, self.desiredZ, self.desiredW)	
				

if __name__ == "__main__":

    rospy.init_node('MissionControlNode')
    wp_mission_control = MissionControl()
    wp_mission_control.run()
