#!/usr/bin/env python
import roslib; roslib.load_manifest('graupner_serial')
import rospy

import math, numpy

from std_msgs.msg import String

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped

from graupner_serial.msg import DesiredTrajectory, MissionWP, MissionStartStop, TrajectoryParametersGUI, TrajectoryParametersBuilding, BuildingTrajectoryStatus

from graupner_serial.srv import GUITrajectoryParameters_Carrot, BuildingTrajectoryParameters_Carrot

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
	MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint



#from mav_msgs import msgMultiDofJointTrajectoryFromPositionYaw

import time
import pygame, sys, os

class TrajectoryGenerator:

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

		self.isTrajectoryPlanned = [False for x in range(self.number_of_UAVs+1)]

	
		#/---ROS Publishers---/#
		self.mission_start_pub = rospy.Publisher('MissionStartStop', MissionStartStop, queue_size=10)
		self.trajectory_status_pub = rospy.Publisher('BuildingTrajectoryStatus', BuildingTrajectoryStatus, queue_size=10)

		#/---ROS Subscribers---/#
		rospy.Subscriber("TrajectoryParametersGUI", TrajectoryParametersGUI, self.GenerateGUITrajectoryCallback)
		rospy.Subscriber("TrajectoryParametersBuilding", TrajectoryParametersBuilding, self.GenerateBuildingTrajectoryCallback)
		rospy.Subscriber("ExecuteBuildingTrajectory", BuildingTrajectoryStatus, self.ExecuteBuildingTrajectoryCallback)

		#/---ROS Services---/#
		self.request_TrajectoryFromGUIService = rospy.ServiceProxy('TrajectoryFromGUIService', GUITrajectoryParameters_Carrot)
		self.request_TrajectoryPlanningService = rospy.ServiceProxy('TrajectoryPlanningService', BuildingTrajectoryParameters_Carrot)
		
		print("TG initialization")

	def run(self):
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			rate.sleep()
		
	def GenerateGUITrajectoryCallback(self, data):
		resp = self.request_TrajectoryFromGUIService(data.selectedUAV, data.selectedUAV_mode, data.desiredX, data.desiredY, data.desiredZ, data.desiredW, data.desiredHS, data.desiredVS, data.desiredHA, data.desiredVA)
		self.planned_trajectory[data.selectedUAV] = resp.multi_dof_trajectory_data

		msgMissionStart = MissionStartStop()
		msgMissionStart.selectedUAV = data.selectedUAV
		msgMissionStart.isPositionHoldActiveFlag = True
		msgMissionStart.isWPActiveFlag = False
		msgMissionStart.isBuildingTrajectoryActiveFlag = False
		msgMissionStart.multi_dof_trajectory_data = self.planned_trajectory[data.selectedUAV]
		self.mission_start_pub.publish(msgMissionStart)

	def GenerateBuildingTrajectoryCallback(self, data):
		selected_UAV = data.selectedUAV
		resp = self.request_TrajectoryPlanningService(data.building_x, data.building_y, data.flight_altitude, data.building_distance, data.trajectory_resolution, data.wp_file_path, data.coord_sys_text, data.plot_flag, data.h_speed, data.v_speed, data.h_acc, data.v_acc)
		self.planned_trajectory[selected_UAV] = resp.multi_dof_trajectory_data

		self.isTrajectoryPlanned[selected_UAV] = True

		msgTrajectoryStatus = BuildingTrajectoryStatus()
		msgTrajectoryStatus.selectedUAV = selected_UAV
		msgTrajectoryStatus.BuildingTrajectoryReady = True
		msgTrajectoryStatus.ExecuteBuildingTrajectory = False
		self.trajectory_status_pub.publish(msgTrajectoryStatus)


	def ExecuteBuildingTrajectoryCallback(self, data):
		selected_UAV = data.selectedUAV
		if self.isTrajectoryPlanned[selected_UAV] and data.ExecuteBuildingTrajectory:
			msgMissionStart = MissionStartStop()
			msgMissionStart.selectedUAV = data.selectedUAV
			msgMissionStart.isPositionHoldActiveFlag = False
			msgMissionStart.isWPActiveFlag = False
			msgMissionStart.isBuildingTrajectoryActiveFlag = True
			msgMissionStart.multi_dof_trajectory_data = self.planned_trajectory[selected_UAV]
			self.mission_start_pub.publish(msgMissionStart)
			print("EXE")

		self.isTrajectoryPlanned[selected_UAV] = False

if __name__ == "__main__":
	rospy.init_node('TrajectoryGeneratorNode')
	trajectory_generator = TrajectoryGenerator()
	trajectory_generator.run()