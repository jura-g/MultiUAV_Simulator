#!/usr/bin/env python
import roslib; roslib.load_manifest('graupner_serial')
import rospy

import math, numpy
#from math import acos
#from math import hypoth

from std_msgs.msg import String

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped

from graupner_serial.msg import DesiredTrajectory, MissionWP, MissionStartStop

from graupner_serial.srv import GUITrajectoryParameters_Carrot

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
	MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint



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

		self.WP_mission_started = False

		self.selected_UAV_name = "UAV"

		self.UAV_names = ["UAV", "red", "blue", "yellow"]
		self.carrot_status = [0 for x in range(self.number_of_UAVs+1)]

		self.trajectory_ref_x_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_y_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_z_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_w_cur = [0.0 for x in range(self.number_of_UAVs+1)]

		self.number_of_WPs = [0 for x in range(self.number_of_UAVs+1)]
		#self.WP_mission_active_flag = [0 for x in range(self.number_of_UAVs+1)]
		#self.building_trajectory_active_flag = [False for x in range(self.number_of_UAVs+1)]
		self.mission_type = [0 for x in range(self.number_of_UAVs+1)]
		self.next_WP = [0 for x in range(self.MAX_WPs)]

		self.wpNumber = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)]
		self.WP_X = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)] 
		self.WP_Y = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)] 
		self.WP_Z = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)]
		self.WP_W = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)] 
		self.WP_LT = [[0 for x in range(self.MAX_WPs)] for y in range(self.number_of_UAVs+1)]

		self.trajectory_ready = [False for x in range(self.number_of_UAVs+1)]
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

		self.planned_trajectory = [MultiDOFJointTrajectory for x in range(self.number_of_UAVs+1)]
		self.trajectory_point_counter = [0 for x in range(self.number_of_UAVs+1)]

	
		#/---ROS Publishers---/#
		self.multi_dof_traj_point_pub = []
		for n in range(self.number_of_UAVs+1):
			temp_pub = rospy.Publisher(self.UAV_names[n]+'/position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=10)
			self.multi_dof_traj_point_pub.append(temp_pub)

		#/---ROS Subscribers---/#
		for n in range(1, self.number_of_UAVs+1):
			rospy.Subscriber(self.UAV_names[n]+"/mavros/global_position/local", Odometry, self.OdometryCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/carrot/trajectory", MultiDOFJointTrajectoryPoint, self.CarrotTrajectoryCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/carrot/status", String, self.CarrotStatusCallback, callback_args=n)
		rospy.Subscriber("MissionfromGUI", MissionWP, self.MissionWPCallback)
		rospy.Subscriber("MissionStartStop", MissionStartStop, self.StartStopCallback)

		#/---ROS Services---/#
		self.request_TrajectoryFromGUIService = rospy.ServiceProxy('TrajectoryFromGUIService', GUITrajectoryParameters_Carrot)

		print("MCN initialization")


	def run(self):
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			for i in range(1, self.number_of_UAVs+1):
				if self.mission_type[i] == 1:
	  				self.SendPositionHoldTrajectoryData(i)
				elif self.mission_type[i] == 2:
					self.SendWPData(i)
				elif self.mission_type[i] == 3:
					self.SendBuildingTrajectoryData(i)
				#print(self.mission_type[i])
			rate.sleep()


	#/* ODOMETRY handlers */

	def OdometryCallback(self, data, UAV_ID):
		self.X_cur_position[0][UAV_ID] = data.pose.pose.position.x
		self.Y_cur_position[0][UAV_ID] = data.pose.pose.position.y
		self.Z_cur_position[0][UAV_ID] = data.pose.pose.position.z
		self.W_cur_orientation[0][UAV_ID] = math.atan2(2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y), 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z))

	def CarrotStatusCallback(self, data, UAV_ID):
		if data.data == "OFF":
			self.carrot_status[UAV_ID] = 0
		elif data.data == "HOLD":
			self.carrot_status[UAV_ID] = 1
		elif data.data == "CARROT_ON_AIR":
			self.carrot_status[UAV_ID] = 2
		
	def CarrotTrajectoryCallback(self, data, UAV_ID):
		self.trajectory_ref_x_cur[UAV_ID] = data.transforms[0].translation.x
		self.trajectory_ref_y_cur[UAV_ID] = data.transforms[0].translation.y
		self.trajectory_ref_z_cur[UAV_ID] = data.transforms[0].translation.z
		self.trajectory_ref_w_cur[UAV_ID] = math.atan2(2.0 * (data.transforms[0].rotation.w * data.transforms[0].rotation.z + data.transforms[0].rotation.x * data.transforms[0].rotation.y), 1.0 - 2.0 * (data.transforms[0].rotation.y * data.transforms[0].rotation.y + data.transforms[0].rotation.z * data.transforms[0].rotation.z))


	#/* MISSION handlers */

	def StartStopCallback(self, data):	
		self.selectedUAV = data.selectedUAV
		if (data.isPositionHoldActiveFlag == True):
			self.mission_type[self.selectedUAV] = 1
			self.planned_trajectory[self.selectedUAV] = data.multi_dof_trajectory_data
			self.trajectory_point_counter[self.selectedUAV] = 0
			print(len(self.planned_trajectory[self.selectedUAV].points))
		elif (data.isWPActiveFlag == True):
			self.mission_type[self.selectedUAV] = 2
			self.next_WP[self.selectedUAV] = 0
			self.trajectory_point_counter[self.selectedUAV] = 0
			self.WP_mission_started = False
		elif (data.isBuildingTrajectoryActiveFlag == True):
			self.mission_type[self.selectedUAV] = 3
			self.planned_trajectory[self.selectedUAV] = data.multi_dof_trajectory_data
			self.trajectory_point_counter[self.selectedUAV] = 0
			print(len(self.planned_trajectory[self.selectedUAV].points))
		else:
			self.mission_type[self.selectedUAV] = 0


	def MissionWPCallback(self, data):
		self.selectedUAV = data.selectedUAV
		self.number_of_WPs[self.selectedUAV] = data.numberofWPs

		#print(self.number_of_WPs[self.selectedUAV])
		print(data.wpX)
		for i in range(self.number_of_WPs[self.selectedUAV]):
			#print(data.wpNumber[i])
			self.wpNumber[self.selectedUAV][i] = data.wpNumber[i]
			self.WP_X[self.selectedUAV][i] = data.wpX[i]
			self.WP_Y[self.selectedUAV][i] = data.wpY[i]
			self.WP_Z[self.selectedUAV][i] = data.wpZ[i]
			self.WP_W[self.selectedUAV][i] = data.wpW[i]
			self.WP_LT[self.selectedUAV][i] = data.wpLT[i]
			#print(self.WP_X[self.selectedUAV][i])

	def SendPositionHoldTrajectoryData(self, UAV_ID):
		if (self.trajectory_point_counter[UAV_ID] < len(self.planned_trajectory[UAV_ID].points)):
			print(self.trajectory_point_counter[UAV_ID])
			temp_point = MultiDOFJointTrajectoryPoint()
			temp_point = self.planned_trajectory[UAV_ID].points[self.trajectory_point_counter[UAV_ID]]
			self.multi_dof_traj_point_pub[UAV_ID].publish(temp_point)
			self.trajectory_point_counter[UAV_ID] = self.trajectory_point_counter[UAV_ID] + 1


	def SendWPData(self, UAV_ID):

		if (self.next_WP[UAV_ID] == 0) and (not self.WP_mission_started):
		

			if self.carrot_status[UAV_ID] == 1:
				desiredX = [0.0 for x in range(2)]
				desiredY = [0.0 for x in range(2)]
				desiredZ = [0.0 for x in range(2)]
				desiredW = [0.0 for x in range(2)]

				self.UAV_mode[0][UAV_ID] = 2

				desiredX[0] = self.trajectory_ref_x_cur[UAV_ID]
				desiredY[0] = self.trajectory_ref_y_cur[UAV_ID]
				desiredZ[0] = self.trajectory_ref_z_cur[UAV_ID]
				desiredW[0] = self.trajectory_ref_w_cur[UAV_ID]
				desiredX[1] = self.WP_X[UAV_ID][0]
				desiredY[1] = self.WP_Y[UAV_ID][0]
				desiredZ[1] = self.WP_Z[UAV_ID][0]
				desiredW[1] = math.radians(self.WP_W[UAV_ID][0])
				#desiredHS = float(self.tbHS_PH.get_text())
				#desiredVS = float(self.tbVS_PH.get_text())
				#desiredHA = float(self.tbHA_PH.get_text())
				#desiredVA = float(self.tbVA_PH.get_text())

				print(self.WP_X[UAV_ID][0])

				resp = self.request_TrajectoryFromGUIService(UAV_ID, self.UAV_mode[0][UAV_ID], desiredX, desiredY, desiredZ, desiredW, 1, 1, 0.5, 0.5)
				self.planned_trajectory[UAV_ID] = resp.multi_dof_trajectory_data
				self.trajectory_ready[UAV_ID] = True
				self.Flight[UAV_ID] = True
				self.WP_mission_started = True
				self.trajectory_point_counter[UAV_ID] = 0				
				
				print("MCN test 1")
	

		if ((self.Flight[UAV_ID]) and (math.hypot((self.WP_X[UAV_ID][self.next_WP[UAV_ID]]-self.X_cur_position[0][UAV_ID]),(self.WP_Y[UAV_ID][self.next_WP[UAV_ID]]-self.Y_cur_position[0][UAV_ID])) < 0.2) and ((self.WP_Z[UAV_ID][self.next_WP[UAV_ID]]-self.Z_cur_position[0][UAV_ID]) < 0.2)):
			self.start_LT[UAV_ID] = rospy.get_time()
			self.Loiter[UAV_ID] = True
			self.Flight[UAV_ID] = False
			print("MCN test 2")
			print(self.start_LT[UAV_ID])

		print(rospy.get_time()-self.start_LT[UAV_ID])
		#print((self.next_WP[UAV_ID] + 2 <= self.number_of_WPs[UAV_ID]))
		print(self.next_WP[UAV_ID])
		print((math.hypot((self.WP_X[UAV_ID][self.next_WP[UAV_ID]]-self.X_cur_position[0][UAV_ID]),(self.WP_Y[UAV_ID][self.next_WP[UAV_ID]]-self.Y_cur_position[0][UAV_ID])) < 0.5) and ((self.WP_Z[UAV_ID][self.next_WP[UAV_ID]]-self.Z_cur_position[0][UAV_ID]) < 0.2))
		if ((self.Loiter[UAV_ID]) and (rospy.get_time() - self.start_LT[UAV_ID] > self.WP_LT[UAV_ID][self.next_WP[UAV_ID]]) and (self.next_WP[UAV_ID] + 2 <= self.number_of_WPs[UAV_ID])): 
			self.Loiter[UAV_ID] = False
			#self.Flight[UAV_ID] = True
			
			if self.carrot_status[UAV_ID] == 1:
				desiredX = [0.0 for x in range(2)]
				desiredY = [0.0 for x in range(2)]
				desiredZ = [0.0 for x in range(2)]
				desiredW = [0.0 for x in range(2)]

				self.UAV_mode[0][UAV_ID] = 2
				self.next_WP[UAV_ID] = self.next_WP[UAV_ID] + 1


				desiredX[0] = self.trajectory_ref_x_cur[UAV_ID]
				desiredY[0] = self.trajectory_ref_y_cur[UAV_ID]
				desiredZ[0] = self.trajectory_ref_z_cur[UAV_ID]
				desiredW[0] = self.trajectory_ref_w_cur[UAV_ID]
				desiredX[1] = self.WP_X[UAV_ID][self.next_WP[UAV_ID]]
				desiredY[1] = self.WP_Y[UAV_ID][self.next_WP[UAV_ID]]
				desiredZ[1] = self.WP_Z[UAV_ID][self.next_WP[UAV_ID]]
				desiredW[1] = math.radians(self.WP_W[UAV_ID][self.next_WP[UAV_ID]])
				#desiredHS = float(self.tbHS_PH.get_text())
				#desiredVS = float(self.tbVS_PH.get_text())
				#desiredHA = float(self.tbHA_PH.get_text())
				#desiredVA = float(self.tbVA_PH.get_text())

				print("MCN test 3")

				print(desiredX)

				resp = self.request_TrajectoryFromGUIService(UAV_ID, self.UAV_mode[0][UAV_ID], desiredX, desiredY, desiredZ, desiredW, 1, 1, 0.5, 0.5)
				self.planned_trajectory[UAV_ID] = resp.multi_dof_trajectory_data
				self.trajectory_ready[UAV_ID] = True
				self.Flight[UAV_ID] = True
				self.trajectory_point_counter[UAV_ID] = 0

		if ((self.Flight[UAV_ID]) and (self.trajectory_ready)):
				if (self.trajectory_point_counter[UAV_ID] < len(self.planned_trajectory[UAV_ID].points)):
					temp_point = MultiDOFJointTrajectoryPoint()
					temp_point = self.planned_trajectory[UAV_ID].points[self.trajectory_point_counter[UAV_ID]]
					self.multi_dof_traj_point_pub[UAV_ID].publish(temp_point)	
					self.trajectory_point_counter[UAV_ID] = self.trajectory_point_counter[UAV_ID] + 1	
				else:
					#self.next_WP[UAV_ID] = self.next_WP[UAV_ID] + 1
					self.trajectory_ready[UAV_ID] = False


	def SendBuildingTrajectoryData(self, UAV_ID):
		if (self.trajectory_point_counter[UAV_ID] < len(self.planned_trajectory[UAV_ID].points)):
			#print("2222")
			print(len(self.planned_trajectory[UAV_ID].points))
			print(self.trajectory_point_counter[UAV_ID])
			temp_point = MultiDOFJointTrajectoryPoint()
			temp_point = self.planned_trajectory[UAV_ID].points[self.trajectory_point_counter[UAV_ID]]
			self.multi_dof_traj_point_pub[UAV_ID].publish(temp_point)	
			self.trajectory_point_counter[UAV_ID] = self.trajectory_point_counter[UAV_ID] + 1	

if __name__ == "__main__":
	rospy.init_node('MissionControlNode')
	mission_control = MissionControl()
	mission_control.run()
