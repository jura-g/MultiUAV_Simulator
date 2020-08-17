#!/usr/bin/env python

#import faulthandler; faulthandler.enable()

import roslib; roslib.load_manifest('graupner_serial')
import rospy
import roslaunch



import math
#from math import acos
#from math import hypoth

import cairo

import cv2

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Transform, Twist

from mavros_msgs.msg import State, HomePosition

from nav_msgs.msg import Odometry

from graupner_serial.msg import DesiredTrajectory, MissionWP, MissionStartStop, BatteryStatus, SelectedUAV, RCchannelData

from std_msgs.msg import String #,Empty

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

#from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

from sensor_msgs.msg import Image, Joy, NavSatFix

from geographic_msgs.msg import GeoPoint

from cv_bridge import CvBridge, CvBridgeError

from graupner_serial.srv import BuildingTrajectoryParameters_Carrot, GUITrajectoryParameters_Carrot

from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandHome

from std_srvs.srv import Empty

#from mav_msgs import msgMultiDofJointTrajectoryFromPositionYaw

import uinput, time
import pygame, sys, os
from pygame.locals import *

import threading

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib, GObject

import matplotlib.pyplot as plt




#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
class stick_state(object):
	def __init__(self, name, stick, channel_num):
		self.name = name							# The name of the stick
		self.stick = stick						  # The stick on the joystick that this stick maps to 
		self.channel_num = channel_num			# Number of channel for stick

		self.min_val = 0.0						  # Minimum stick value
		self.max_val = 255.0						# Maximum stick value
		self.active_up = False					  # True if up key is held pressed
		self.active_down = False					# True if down key is held pressed
		self.zero = 0.0
		self.val = self.zero						# Stick value at initialization at zero position
		self.emit_val = int(self.val)
		self.display_ready = False				  # Whether optional display params have been set
		self.display_height = 0					 # Height on the display screen 
		self.display_width = 0					  # Width on the display screen 
		self.display_hor = True					 # Whether the display bar is horizontal, else vertical
		self.display_bar_g = []
		self.display_bar_b = []

	def emit(self, device):
		# emit effeciently
		if abs(int(round(self.val)) - int(self.emit_val)) > 0.001:
			self.emit_val = int(round(self.val))
			device.emit(self.stick, int(self.emit_val), syn=False)

	def update_stick(self, device, CH1, CH2, CH3, CH4, CH5, CH6):
		if self.channel_num == 1:
			self.val = (CH1-1100)*255/800
		if self.channel_num == 2:
			self.val = (CH2-1100)*255/800
		if self.channel_num == 3:
			self.val = (CH3-1100)*255/800
		if self.channel_num == 4:
			self.val = (CH4-1100)*255/800
		if self.channel_num == 5:
			self.val = (CH5-1100)*255/800
		if self.channel_num == 6:
			self.val = (CH6-1100)*255/800
		self.emit(device)

class app_main:
	def __init__(self):
		#threading.Thread.__init__(self)

		self.number_of_UAVs = 3
		self.number_of_modes = 6	# [0] - manual mode
									# [1] - hold position
									# [2] - waypoint mission
									# [3] - point of interest mode
									# [4] - follow mode
									# [5] - trajectory mode

		self.RCController_status = 'N/A'
		self.CH1 = 0
		self.CH2 = 0
		self.CH3 = 0
		self.CH4 = 0
		self.CH5 = 1000
		self.CH6 = 1100

		self.selected_UAV = 0
		self.selected_UAV_name = "UAV"

		self.UAV_names = ["UAV", "red", "blue", "yellow"]

		self.UAV_mode = [[1 for x in range(self.number_of_UAVs+1)] for y in range(2)]

		self.UAV_state_mode = [0 for x in range(self.number_of_UAVs+1)]
		self.UAV_state_armed = [False for x in range(self.number_of_UAVs+1)]
		self.carrot_status = [0 for x in range(self.number_of_UAVs+1)]

		self.home_position_global_position_latitude = [0.0 for x in range(self.number_of_UAVs+1)]
		self.home_position_global_position_longitude = [0.0 for x in range(self.number_of_UAVs+1)]
		self.home_position_global_position_altitude = [0.0 for x in range(self.number_of_UAVs+1)]

		self.home_position_local_position = [Point for x in range(self.number_of_UAVs+1)]
		self.home_position_local_orientation = [Quaternion for x in range(self.number_of_UAVs+1)]

		self.X_cur_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Y_cur_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Z_cur_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.W_cur_orientation = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.cur_odometry_time = [[0.0 for x in range(self.number_of_UAVs+1)] for y in range(2)]

		self.X_cur_lin_vel = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Y_cur_lin_vel = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Z_cur_lin_vel = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]

		self.H_speed = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.V_speed = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]

		self.X_old_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Y_old_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Z_old_position = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.W_old_orientation = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.old_odometry_time = [[0.0 for x in range(self.number_of_UAVs+1)] for y in range(2)]

		self.first_odometry_callback = [True for x in range(self.number_of_UAVs+1)]

		self.trajectory_ref_x_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_y_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_z_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_w_cur = [0.0 for x in range(self.number_of_UAVs+1)]


		self.lat_cur_global = [0.0 for x in range(self.number_of_UAVs+1)]
		self.long_cur_global = [0.0 for x in range(self.number_of_UAVs+1)]
		self.alt_cur_global = [0.0 for x in range(self.number_of_UAVs+1)]

		self.odo_counter = [0 for x in range(self.number_of_UAVs+1)]

		self.X_home_point = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Y_home_point = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.Z_home_point = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]

		#self.desiredX = [0.0 for x in range(90)]
		#self.desiredY = [0.0 for x in range(90)]
		#self.desiredZ = [0.0 for x in range(90)]
		#self.desiredW = [0.0 for x in range(90)]

		self.new_pos_counter = [0 for x in range(self.number_of_UAVs+1)]

		self.R_X_cur_position = 0.0
		self.R_Y_cur_position = 0.0
		self.R_Z_cur_position = 0.0
		self.R_W_cur_orientation = 0.0

		self.max_batt_data = [[100 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.new_batt_data = [[0 for x in range(self.number_of_UAVs+1)] for y in range(2)]
		self.UAV_batt_data = [[1.0 for x in range(self.number_of_UAVs+1)] for y in range(2)]

		self.R_UAV_batt_data = [0 for x in range(self.number_of_UAVs+1)]

		self.WP_num = 0
		self.UAV_WP = 1
		self.BUILDING_POINT_num = 0
		self.coord_sys = 0
		self.planned_trajectory = [MultiDOFJointTrajectory for x in range(self.number_of_UAVs+1)]
		self.trajectory_point_counter = [0 for x in range(self.number_of_UAVs+1)]
		self.isBuildingTrajectoryReady = [False for x in range(self.number_of_UAVs+1)]


		self.POI_number_of_parameters = 9	# [0] - point X coord
											# [1] - point Y coord
											# [2] - point Z coord
											# [3] - radius
											# [4] - altitude
											# [5] - velocity
											# [6] - direction
											# [7] - angle
											# [8] - timer
		self.POI_param = [[[0 for x in range(self.POI_number_of_parameters)] for y in range(self.number_of_UAVs+1)] for z in range(2)]
		for x in range(self.number_of_UAVs+1):
			self.POI_param[0][x][6] = -1
		self.POI_direction = -1

		self.follow_mode = 1
		self.leader_UAV = [0 for x in range(self.number_of_UAVs+1)]
		self.new_leader_UAV = 0
		self.FOLLOW_number_of_parameters = 3	# [0] - follow distace
												# [1] - follow angle
												# [2] - follow altitude
		self.FOLLOW_param = [[0 for x in range(self.FOLLOW_number_of_parameters)] for y in range(self.number_of_UAVs+1)]

		self.gladefile = roslib.packages.get_pkg_dir('graupner_serial')+"/scripts/MultiUAV_GUI_Node_Mavros.glade"
		self.builder = Gtk.Builder()
		self.builder.add_from_file(self.gladefile)
		self.window = self.builder.get_object("window1")
		self.window.connect("delete-event", self.onDeleteWindow)
		self.window.connect("key-press-event", self.onKeyPressed)

		#/* alignments */
		self.al23 = self.builder.get_object('alignment23')

		#/* boxes */
		self.vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
		self.hbox = [Gtk.Box(spacing=10) for x in range(self.number_of_UAVs+1)]
		for x in range(self.number_of_UAVs+1):
			self.hbox[x].set_homogeneous(False)

		self.al23.add(self.vbox)
		self.lbTELEM_ID = [Gtk.Label(str(x)) for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_MODE = [Gtk.Label("NULL") for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_X = [Gtk.Label("0") for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_Y = [Gtk.Label("0") for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_Z = [Gtk.Label("0") for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_HEADING = [Gtk.Label("0") for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_HS = [Gtk.Label("0") for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_VS = [Gtk.Label("0") for x in range(self.number_of_UAVs+1)]
		self.lbTELEM_BATTERY = [Gtk.Label("0") for x in range(self.number_of_UAVs+1)] 
		self.pbTELEM_BATTERY = [Gtk.ProgressBar() for x in range(self.number_of_UAVs+1)]

		for x in range(self.number_of_UAVs+1):
			self.lbTELEM_ID[x].set_width_chars(10)
			self.lbTELEM_MODE[x].set_width_chars(10)
			self.lbTELEM_X[x].set_width_chars(10)
			self.lbTELEM_Y[x].set_width_chars(10)
			self.lbTELEM_Z[x].set_width_chars(10)
			self.lbTELEM_HEADING[x].set_width_chars(10)
			self.lbTELEM_HS[x].set_width_chars(10)
			self.lbTELEM_VS[x].set_width_chars(10)
			self.lbTELEM_BATTERY[x].set_width_chars(10)
			self.pbTELEM_BATTERY[x].set_show_text(False)
			self.pbTELEM_BATTERY[x].set_size_request(2, 20) 	
		for n in range(1, self.number_of_UAVs+1):
			self.vbox.pack_start(self.hbox[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_ID[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_MODE[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_X[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_Y[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_Z[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_HEADING[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_HS[n], True, True, 0)
			self.hbox[n].pack_start(self.lbTELEM_VS[n], True, True, 0)
			self.hbox[n].pack_start(self.pbTELEM_BATTERY[n], True, True, 0)


		self.window.show_all()


		#/* level bar */
		self.lbS1 = self.builder.get_object('levelbarS1')
		self.lbS2_1 = self.builder.get_object('levelbarS2_1')  
		self.lbS2_2 = self.builder.get_object('levelbarS2_2')  
		self.lbS3_1 = self.builder.get_object('levelbarS3_1')
		self.lbS3_2 = self.builder.get_object('levelbarS3_2')
		self.lbS4_1 = self.builder.get_object('levelbarS4_1')
		self.lbS4_2 = self.builder.get_object('levelbarS4_2')
		self.lb5 = self.builder.get_object('levelbar5')
		self.lb6 = self.builder.get_object('levelbar6')
		self.lb7 = self.builder.get_object('levelbar7')
		#self.lb8 = self.builder.get_object('levelbar8')
		#self.lb9 = self.builder.get_object('levelbar9')

		#/* progress bar */
		self.pb1 = self.builder.get_object('progressbar1')
		self.pb2 = self.builder.get_object('progressbar2')

		#/* label */
		self.lX = self.builder.get_object('labelX')
		self.lY = self.builder.get_object('labelY')
		self.lZ = self.builder.get_object('labelZ')
		self.lW = self.builder.get_object('labelW')
		self.lPH_STAUTS = self.builder.get_object('labelNEWPOSSTATUS')
		self.lFdis = self.builder.get_object('labelFDISTANCE')
		self.lFang = self.builder.get_object('labelFANGLE')
		self.lFalt = self.builder.get_object('labelFALTITUDE')
		#self.lAM = self.builder.get_object('labelAUTMODE')
		self.lTEST = self.builder.get_object('labelTEST')
		self.lBATT = self.builder.get_object('labelBATT')
	
		self.lMODE = self.builder.get_object('labelMODE')
		self.lARMED = self.builder.get_object('labelARMED')
		self.lCARROTSTATUS = self.builder.get_object('labelCARROTSTATUS')
		self.lGPSLOCK = self.builder.get_object('labelGPSLOCK')
		self.lLOCALX = self.builder.get_object('labelLOCALX')
		self.lLOCALY = self.builder.get_object('labelLOCALY')
		self.lLOCALZ = self.builder.get_object('labelLOCALZ')
		self.lLATITUDE = self.builder.get_object('labelLATITUDE')
		self.lLONGITUDE = self.builder.get_object('labelLONGITUDE')
		self.lALTITUDE = self.builder.get_object('labelALTITUDE')
		self.lHEADING = self.builder.get_object('labelHEADING')
		self.lHS = self.builder.get_object('S_labelHS')
		self.lVS = self.builder.get_object('S_labelVS')
		self.lHP_LATITUDE = self.builder.get_object('labelHOMEPOINTLATITUDE')
		self.lHP_LONGITUDE = self.builder.get_object('labelHOMEPOINTLONGITUDE')
		self.S_lTRAJSTATUS = self.builder.get_object('S_labelTRAJECTORYSTATUS')
	
		#/* buttons */
		self.bLAND = self.builder.get_object('buttonLAND')
		self.bLAND.connect("clicked", self.on_buttonLAND_clicked)
		self.bTAKEOFF = self.builder.get_object('buttonTAKEOFF')
		self.bTAKEOFF.connect("clicked", self.on_buttonTAKEOFF_clicked)
		self.bSEND_NEW_POS = self.builder.get_object('buttonSENDNEWPOS')
		self.bSEND_NEW_POS.connect("clicked", self.on_buttonSEND_NEW_POS_clicked)
		self.bHOLD_POS = self.builder.get_object('buttonHOLDPOS')
		self.bHOLD_POS.connect("clicked", self.on_buttonHOLD_POS_clicked)
		self.bRETURN_HOME = self.builder.get_object('buttonRETURNHOME')
		self.bRETURN_HOME.connect("clicked", self.on_buttonRETURN_HOME_clicked)

		self.bARMM = self.builder.get_object('buttonARMMOTORS')
		self.bARMM.connect("clicked", self.on_buttonARMMOTORS_clicked)
		self.bCHANGEMODE = self.builder.get_object('buttonCHANGEMODE')
		self.bCHANGEMODE.connect("clicked", self.on_buttonCHANGEMODE_clicked)
		self.bCARROTHOLD = self.builder.get_object('buttonCARROTHOLD')
		self.bCARROTHOLD.connect("clicked", self.on_buttonCARROTHOLD_clicked)
		self.bHOMERESET = self.builder.get_object('buttonHOMERESET')
		self.bHOMERESET.connect("clicked", self.on_buttonHOMERESET_clicked)
		self.bHOMESYNC = self.builder.get_object('buttonHOMESYNC')
		self.bHOMESYNC.connect("clicked", self.on_buttonHOMESYNC_clicked)

		self.bADD_WP = self.builder.get_object('buttonADDWP')
		self.bADD_WP.connect("clicked", self.on_buttonADD_WP_clicked)
		self.bREMOVE_WP = self.builder.get_object('buttonREMOVEWP')
		self.bREMOVE_WP.connect("clicked", self.on_buttonREMOVE_WP_clicked)
		self.bUPLOAD_WP = self.builder.get_object('buttonUPLOADWP')
		self.bUPLOAD_WP.connect("clicked", self.on_buttonUPLOAD_WP_clicked)
		self.bSTART_WP = self.builder.get_object('buttonSTARTWP')
		self.bSTART_WP.connect("clicked", self.on_buttonSTART_WP_clicked)
		self.bWP_LOAD_MISSION = self.builder.get_object('buttonWPLOADMISSION')
		self.bWP_LOAD_MISSION.connect("clicked", self.on_buttonWP_LOAD_MISSION_clicked)

		self.bFOLLOW = self.builder.get_object('buttonFOLLOW')
		self.bFOLLOW.connect("clicked", self.on_buttonFOLLOW_clicked)

		self.bPOISTART = self.builder.get_object('buttonPOISTART')
		self.bPOISTART.connect("clicked", self.on_buttonPOISTART_clicked)

		self.bPLAN_TRAJECTORY = self.builder.get_object('buttonPLAN_TRAJECTORY')
		self.bPLAN_TRAJECTORY.connect("clicked", self.on_buttonPLAN_TRAJECTORY_clicked)
		self.bLOAD_BUILDING_POINTS = self.builder.get_object('buttonLOAD_BUILDING_FILE')
		self.bLOAD_BUILDING_POINTS.connect("clicked", self.on_buttonLOAD_BUILDING_FILE_clicked)
		self.bADD_BUILDING_POINT = self.builder.get_object('buttonADD_BUILDING_POINT')
		self.bADD_BUILDING_POINT.connect("clicked", self.on_buttonADD_BUILDING_POINT_clicked)
		self.bREMOVE_BUILDING_POINT = self.builder.get_object('buttonREMOVE_BUILDING_POINT')
		self.bREMOVE_BUILDING_POINT.connect("clicked", self.on_buttonREMOVE_BUILDING_POINT_clicked)
		self.bEXECUTE_TRAJECTORY = self.builder.get_object('buttonEXECUTE_TRAJECTORY')
		self.bEXECUTE_TRAJECTORY.connect("clicked", self.on_buttonEXECUTE_TRAJECTORY_clicked) 


		#/* radiobuttons */
		self.rbPOICW = self.builder.get_object('radiobuttonPOICW')
		self.rbPOICCW = self.builder.get_object('radiobuttonPOICCW')
		self.rbPOICW.connect("toggled", self.on_radiobuttonPOICW_toggled)

		self.rbWP1 = self.builder.get_object('radiobuttonWP1')
		self.rbWP2 = self.builder.get_object('radiobuttonWP2')
		self.rbWP3 = self.builder.get_object('radiobuttonWP3')
		self.rbWP1.connect("toggled", self.on_radiobuttonWP_UAV_toggled, 1)
		self.rbWP2.connect("toggled", self.on_radiobuttonWP_UAV_toggled, 2)
		self.rbWP3.connect("toggled", self.on_radiobuttonWP_UAV_toggled, 3)

		self.rb1 = self.builder.get_object('radiobutton1')
		self.rb2 = self.builder.get_object('radiobutton2')
		self.rb3 = self.builder.get_object('radiobutton3')
		self.rb1.connect("toggled", self.on_radiobuttonUAV_toggled, 1)
		self.rb2.connect("toggled", self.on_radiobuttonUAV_toggled, 2)
		self.rb3.connect("toggled", self.on_radiobuttonUAV_toggled, 3)
		self.rbFC = self.builder.get_object('radiobuttonFC')
		self.rbFN = self.builder.get_object('radiobuttonFN')
		self.rbFC.connect("toggled", self.on_radiobuttonFMODE_toggled)

		#/* switch */
		self.swCARROT_CONTOLLER = self.builder.get_object('switchCARROTCONTOLLER')
		self.swCARROT_CONTOLLER.connect("notify::active", self.on_switchCARROTCONTOLLER_activated)	

		#/* text box */
		self.tbX = self.builder.get_object('entryX')
		self.tbY = self.builder.get_object('entryY')
		self.tbZ = self.builder.get_object('entryZ')
		self.tbW = self.builder.get_object('entryW')
		self.tbHS_PH = self.builder.get_object('entryHS_POSHOLD')
		self.tbVS_PH = self.builder.get_object('entryVS_POSHOLD')
		self.tbHA_PH = self.builder.get_object('entryHA_POSHOLD')
		self.tbVA_PH = self.builder.get_object('entryVA_POSHOLD')
		self.tbPOIX = self.builder.get_object('entryPOIX')
		self.tbPOIY = self.builder.get_object('entryPOIY')
		self.tbPOIZ = self.builder.get_object('entryPOIZ')
		self.tbPOIR = self.builder.get_object('entryPOIRAD')
		self.tbPOIA = self.builder.get_object('entryPOIALT')
		self.tbPOIV = self.builder.get_object('entryPOIVEL')
		self.tbFdis = self.builder.get_object('entryFDISTANCE')
		self.tbFang = self.builder.get_object('entryFANGLE')
		self.tbFalt = self.builder.get_object('entryFALTITUDE')
		self.S_tbRTH_ALT = self.builder.get_object('S_entryRTH_ALT')

		self.tbTRAJ_ALT = self.builder.get_object('entryTRAJ_ALT')
		self.tbBUIL_DIST = self.builder.get_object('entryBUIL_DIST')
	  	self.tbWP_RES = self.builder.get_object('entryWP_RES')
		self.tbWP_PATH = self.builder.get_object('entryWP_PATH')
		self.tbHS_TP = self.builder.get_object('entryHS_TRAJPLANNER')
		self.tbVS_TP = self.builder.get_object('entryVS_TRAJPLANNER')
		self.tbHA_TP = self.builder.get_object('entryHA_TRAJPLANNER')
		self.tbVA_TP = self.builder.get_object('entryVA_TRAJPLANNER')

		#/* combo box text */
		self.cbtRCCONTROLLER = self.builder.get_object('comboboxtextRCCONTROLLER')
		self.cbtRCCONTROLLER.connect("changed", self.on_comboboxtextRCCONTROLLER_changed)
		self.cbtCOORDSYS = self.builder.get_object('comboboxtextCOORD_SYS')
		self.cbtUAVMODE = self.builder.get_object('comboboxtextUAV_MODE')

		#/* scale */
		self.scTHROTTLE = self.builder.get_object('scaleTHROTTLE')
		#self.scTHROTTLE.connect("value-changed", self.scaleTHROTTLE_moved)
		self.scPITCH = self.builder.get_object('scalePITCH')
		#self.scPITCH.connect("value-changed", self.scalePITCH_moved)
		self.scROLL = self.builder.get_object('scaleROLL')
		#self.scROLL.connect("value-changed", self.scaleROLL_moved)
		self.scYAW = self.builder.get_object('scaleYAW')
		#self.scYAW.connect("value-changed", self.scaleYAW_moved)

		#/* tree view */
		self.twWP = self.builder.get_object('treeviewWP')
		self.liststore1 = Gtk.ListStore(int, float, float, float, float, float)
		self.twWP.set_model(self.liststore1)

		for i, column_title in enumerate(["WP #", "X", "Y", "Z", "Heading", "Loiter time"]):
			renderer = Gtk.CellRendererText()
			renderer.set_property("editable", True)
			column = Gtk.TreeViewColumn(column_title, renderer, text=i)
			self.twWP.append_column(column)

		if column_title == "X":
			renderer.connect("edited", self.WP_edited_X)
		elif column_title == "Y": 
			renderer.connect("edited", self.WP_edited_Y)
		elif column_title == "Z": 
				renderer.connect("edited", self.WP_edited_Z)
		elif column_title == "Heading": 
			renderer.connect("edited", self.WP_edited_W)
		elif column_title == "Loiter time": 
			renderer.connect("edited", self.WP_edited_LT)


		self.twTRAJ = self.builder.get_object('treeviewTRAJ')
		self.liststore4 = Gtk.ListStore(int, float, float)
		self.twTRAJ.set_model(self.liststore4)

		for i, column_title in enumerate(["POINT #", "X", "Y"]):
			renderer = Gtk.CellRendererText()
			renderer.set_property("editable", True)
			column = Gtk.TreeViewColumn(column_title, renderer, text=i)
			self.twTRAJ.append_column(column)

		if column_title == "X":
			renderer.connect("edited", self.TRAJ_edited_X)
		elif column_title == "Y": 
			renderer.connect("edited", self.TRAJ_edited_Y)

		#/* notebook */
		self.nb1 = self.builder.get_object('notebook1')

		#/* image */

		#/* drawing area */
		self.S_daHEADING = self.builder.get_object('S_drawingareaHEADING')

		self.arrow_surface_1 = cairo.ImageSurface.create_from_png("/home/jura/catkin_ws/src/MultiUAV_Simulator/scripts/images/S_arrow_4-100.png")
		self.arrow_surface_blank_1 = cairo.ImageSurface.create_from_png("/home/jura/catkin_ws/src/MultiUAV_Simulator/scripts/images/background_XY_4.png")
		self.arrow_surface_2 = cairo.ImageSurface.create_from_png("/home/jura/catkin_ws/src/MultiUAV_Simulator/scripts/images/S_arrow_4-100.png")
		self.arrow_surface_blank_2 = cairo.ImageSurface.create_from_png("/home/jura/catkin_ws/src/MultiUAV_Simulator/scripts/images/background_N_2.png")
		#self.arrow_context_1 = cairo.Context(arrow_surface_1)

		events = (
			uinput.BTN_JOYSTICK,
			uinput.ABS_X + (0, 255, 0, 0),
			uinput.ABS_Y + (0, 255, 0, 0),
			uinput.ABS_THROTTLE + (0, 255, 0, 0),
			uinput.ABS_RUDDER + (0, 255, 0, 0),
			uinput.ABS_WHEEL + (0, 255, 0, 0),
			uinput.ABS_GAS + (0, 255, 0, 0),
			)

		self.device = uinput.Device(events)

		self.sticks = []

		#__create sticks__
		self.roll_stick = stick_state('Roll', uinput.ABS_X, 2)
		self.sticks.append(self.roll_stick)
		self.pitch_stick = stick_state('Pitch', uinput.ABS_Y, 3)
		self.sticks.append(self.pitch_stick)
		self.thr_stick = stick_state('Throttle', uinput.ABS_THROTTLE, 1)
		self.sticks.append(self.thr_stick)
		self.rud_stick = stick_state('Yaw', uinput.ABS_RUDDER, 4)
		self.sticks.append(self.rud_stick)
		self.mode_stick_1 = stick_state('Mode_1', uinput.ABS_WHEEL, 5)
		self.sticks.append(self.mode_stick_1)
		self.mode_stick_2 = stick_state('Mode_2', uinput.ABS_GAS, 6)
		self.sticks.append(self.mode_stick_2)
	
		rospy.init_node('MultiUAV_GUI_Node')

		#/---ROS Publishers---/#
		self.carrot_activate_pub = []
		self.simRC_pub = []
		self.home_point_pub = []
		for n in range(self.number_of_UAVs+1):
			temp_pub = rospy.Publisher(self.UAV_names[n]+'/joy', Joy, queue_size=10)
			time.sleep(0.02)
			self.carrot_activate_pub.append(temp_pub)
			temp_pub = rospy.Publisher(self.UAV_names[n]+'/joy', Joy, queue_size=10)
			time.sleep(0.02)
			self.simRC_pub.append(temp_pub)
			temp_pub = rospy.Publisher(self.UAV_names[n]+'/mavros/global_position/home', HomePosition, queue_size=10, latch=True)
			time.sleep(0.02)
			self.home_point_pub.append(temp_pub)
		self.mission_start_pub = rospy.Publisher('MissionStartStop', MissionStartStop, queue_size=10)
		self.mission_wp_pub = rospy.Publisher('MissionfromGUI', MissionWP, queue_size=10)
		
		#/---ROS Subscribers---/#
		rospy.Subscriber("GraupnerRCchannels", RCchannelData, self.RCchannelCallback)

		for n in range(1, self.number_of_UAVs+1):
			rospy.Subscriber(self.UAV_names[n]+"/mavros/global_position/local", Odometry, self.OdometryCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/mavros/global_position/global", NavSatFix, self.GlobalOdometryCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/mavros/state", State, self.StateCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/carrot/status", String, self.CarrotStatusCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/mavros/global_position/home", HomePosition, self.HomePositionCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/carrot/trajectory", MultiDOFJointTrajectoryPoint, self.CarrotTrajectoryCallback, callback_args=n)
		rospy.Subscriber("BatteryStatusforGUI", BatteryStatus, self.BatteryCallback)

		#/---ROS Services---/#
		self.request_TrajectoryPlanningService = rospy.ServiceProxy('TrajectoryPlanningService', BuildingTrajectoryParameters_Carrot)
		self.request_TrajectoryFromGUIService = rospy.ServiceProxy('TrajectoryFromGUIService', GUITrajectoryParameters_Carrot)
		self.request_SetModeService = []
		self.request_ArmMotorsService = []
		self.request_TakeOffService = []
		self.request_SetHomePosition = []
		self.request_CarrotHoldService = []
		for n in range(self.number_of_UAVs+1):
			temp_srv = rospy.ServiceProxy(self.UAV_names[n]+'/mavros/set_mode', SetMode)
			self.request_SetModeService.append(temp_srv)
			temp_srv = rospy.ServiceProxy(self.UAV_names[n]+'/mavros/cmd/arming', CommandBool)
			self.request_ArmMotorsService.append(temp_srv)
			temp_srv = rospy.ServiceProxy(self.UAV_names[n]+'/mavros/cmd/takeoff', CommandTOL)
			self.request_TakeOffService.append(temp_srv) 
			temp_srv = rospy.ServiceProxy(self.UAV_names[n]+'/mavros/cmd/set_home', CommandHome)
			self.request_SetHomePosition.append(temp_srv)
			temp_srv = rospy.ServiceProxy(self.UAV_names[n]+'/position_hold', Empty)
			self.request_CarrotHoldService.append(temp_srv)

		time.sleep(0.5)
	#---------------------------------------------------------------------------------------------------------------------------------------------------------#
	#/* signals handlers */
	def onDeleteWindow(self, *args):
		Gtk.main_quit(*args)

	def onKeyPressed(self, widget, event):
		#self.S_lGPSLOCK.set_text(str(event.keyval))
		if event.keyval == 97:
			self.CH5 = 1100
			self.selected_UAV = 1
			self.selected_UAV_name = self.UAV_names[1]
		if event.keyval == 98:
			self.CH5 = 1500
			self.selected_UAV = 2
			self.selected_UAV_name = self.UAV_names[2]
		if event.keyval == 99:
			self.CH5 = 1900
			self.selected_UAV = 3
			self.selected_UAV_name = self.UAV_names[3]

		self.lHP_LATITUDE.set_text(str(round(self.home_position_global_position_latitude[self.selected_UAV], 7)))
		self.lHP_LONGITUDE.set_text(str(round(self.home_position_global_position_longitude[self.selected_UAV], 7)))

		if self.carrot_status[self.selected_UAV] == 0:
			self.swCARROT_CONTOLLER.set_active(False)
		else:
			self.swCARROT_CONTOLLER.set_active(True)

	def on_comboboxtextRCCONTROLLER_changed(self, combo):
		self.RCController_status = combo.get_active_text()
		
	#---------------------------------------------------------------------------------------------------------------------------------------------------------#
	#/* ODOMETRY and STATE handlers */
	def OdometryCallback(self, data, UAV_ID):
		row = Gtk.TreePath(UAV_ID-1)

		if self.first_odometry_callback[UAV_ID]:
			self.old_odometry_time[0][UAV_ID] = data.header.stamp.secs + float(data.header.stamp.nsecs)/1000000000
			self.first_odometry_callback[UAV_ID] = False
		
		if (self.cur_odometry_time[0][UAV_ID] - self.old_odometry_time[0][UAV_ID]) > 1:
 			self.X_old_position[0][UAV_ID] = self.X_cur_position[0][UAV_ID]
			self.Y_old_position[0][UAV_ID] = self.Y_cur_position[0][UAV_ID]
			self.Z_old_position[0][UAV_ID] = self.Z_cur_position[0][UAV_ID]
			self.W_old_orientation[0][UAV_ID] = self.W_cur_orientation[0][UAV_ID]
			self.old_odometry_time[0][UAV_ID] = self.cur_odometry_time[0][UAV_ID]


		self.X_cur_position[0][UAV_ID] = data.pose.pose.position.x
		self.Y_cur_position[0][UAV_ID] = data.pose.pose.position.y
		self.Z_cur_position[0][UAV_ID] = data.pose.pose.position.z

		self.W_cur_orientation[0][UAV_ID] = math.atan2(2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y), 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z))

		self.cur_odometry_time[0][UAV_ID] = data.header.stamp.secs + float(data.header.stamp.nsecs)/1000000000
		self.X_cur_lin_vel[0][UAV_ID] = data.twist.twist.linear.x
		self.Y_cur_lin_vel[0][UAV_ID] = data.twist.twist.linear.y
		self.Z_cur_lin_vel[0][UAV_ID] = -data.twist.twist.linear.z

		self.H_speed[0][UAV_ID] = math.hypot(self.X_cur_lin_vel[0][UAV_ID], self.Y_cur_lin_vel[0][UAV_ID])
		self.V_speed[0][UAV_ID] = self.Z_cur_lin_vel[0][UAV_ID]


	def GlobalOdometryCallback(self, data, UAV_ID):
		self.lat_cur_global[UAV_ID] = data.latitude
		self.long_cur_global[UAV_ID] = data.longitude
		self.alt_cur_global[UAV_ID] = data.altitude

	def StateCallback(self, data, UAV_ID):
		if data.mode == "LAND":
			self.UAV_state_mode[UAV_ID] = 0
		elif data.mode == "ALT_HOLD":
			self.UAV_state_mode[UAV_ID] = 1
		elif data.mode == "GUIDED":
			self.UAV_state_mode[UAV_ID] = 2
		elif data.mode == "LOITER":
			self.UAV_state_mode[UAV_ID] = 3
		elif data.mode == "GUIDED_NOGPS":
			self.UAV_state_mode[UAV_ID] = 4

		self.UAV_state_armed[UAV_ID] = data.armed

	def CarrotStatusCallback(self, data, UAV_ID):
		if data.data == "OFF":
			self.carrot_status[UAV_ID] = 0			
			msgMissionStart = MissionStartStop()
			msgMissionStart.selectedUAV = UAV_ID
			msgMissionStart.isPositionHoldActiveFlag = False
			msgMissionStart.isWPActiveFlag = False
			msgMissionStart.isBuildingTrajectoryActiveFlag = False
			self.mission_start_pub.publish(msgMissionStart)
		elif data.data == "HOLD":
			self.carrot_status[UAV_ID] = 1
		elif data.data == "CARROT_ON_AIR":
			self.carrot_status[UAV_ID] = 2

	def HomePositionCallback(self, data, UAV_ID):
		self.home_position_global_position_latitude[UAV_ID] = data.geo.latitude
		self.home_position_global_position_longitude[UAV_ID] = data.geo.longitude
		self.home_position_global_position_altitude[UAV_ID] = data.geo.altitude
		self.home_position_local_position[UAV_ID] = data.position
		self.home_position_local_orientation[UAV_ID] = data.orientation

		self.lHP_LATITUDE.set_text(str(round(self.home_position_global_position_latitude[self.selected_UAV], 7)))
		self.lHP_LONGITUDE.set_text(str(round(self.home_position_global_position_longitude[self.selected_UAV], 7)))

	def CarrotTrajectoryCallback(self, data, UAV_ID):
		self.trajectory_ref_x_cur[UAV_ID] = data.transforms[0].translation.x
		self.trajectory_ref_y_cur[UAV_ID] = data.transforms[0].translation.y
		self.trajectory_ref_z_cur[UAV_ID] = data.transforms[0].translation.z
		self.trajectory_ref_w_cur[UAV_ID] = math.atan2(2.0 * (data.transforms[0].rotation.w * data.transforms[0].rotation.z + data.transforms[0].rotation.x * data.transforms[0].rotation.y), 1.0 - 2.0 * (data.transforms[0].rotation.y * data.transforms[0].rotation.y + data.transforms[0].rotation.z * data.transforms[0].rotation.z))

	

#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#   /* RC signals handlers */
	def RCchannelCallback(self, data):
		if self.RCController_status == 'GraupnerRC':
			self.CH1 = data.CH1
			self.CH2 = data.CH2
			self.CH3 = data.CH3
			self.CH4 = data.CH4
			self.CH5 = data.CH5
			self.CH6 = data.CH6
#---------------------------------------------------------------------------------------------------------------------------------------------------------#
	#/* ARM MOTORS function */
	def on_buttonARMMOTORS_clicked(self, button):
		resp = self.request_ArmMotorsService[self.selected_UAV](True)

	#/* CHANGE MODE function */	
	def on_buttonCHANGEMODE_clicked(self, button):

		selected_mode = self.cbtUAVMODE.get_active_text()
		print(selected_mode)
		resp = self.request_SetModeService[self.selected_UAV](0, selected_mode)

	#/* CARROT ACTIVATION function */ 
	def on_switchCARROTCONTOLLER_activated(self, switch, active):
		if (switch.get_active()) and (self.RCController_status == 'N/A'):
			msgCarrotActivate = Joy()
			msgCarrotActivate.axes = [0,0,0,0,0,0]
			msgCarrotActivate.buttons = [0,0,0,0,0,1]
			self.carrot_activate_pub[self.selected_UAV].publish(msgCarrotActivate)
			print('Switch ON')
		elif (self.RCController_status == 'N/A'):
			msgCarrotActivate = Joy()
			msgCarrotActivate.axes = [0,0,0,0,0,0]
			msgCarrotActivate.buttons = [0,0,0,0,0,0]
			self.carrot_activate_pub[self.selected_UAV].publish(msgCarrotActivate)
			resp = self.request_SetModeService[self.selected_UAV](0, "GUIDED_NOGPS")

	#/* CHANGE MODE function */
	def on_buttonCARROTHOLD_clicked(self, button):
		resp = self.request_CarrotHoldService[self.selected_UAV]()
	
	#/* HOME POINT functions */
	def on_buttonHOMERESET_clicked(self, button):
		home_latitude = self.lat_cur_global[self.selected_UAV]
		home_longitude = self.long_cur_global[self.selected_UAV]
		home_altitude = self.alt_cur_global[self.selected_UAV]
		msgHomePos = HomePosition()
		msgHomePos.geo.latitude = home_latitude
		msgHomePos.geo.longitude = home_longitude
		msgHomePos.geo.altitude = home_altitude
		self.home_point_pub[self.selected_UAV].publish(msgHomePos)

		self.lHP_LATITUDE.set_text(str(round(self.home_position_global_position_latitude[self.selected_UAV], 7)))
		self.lHP_LONGITUDE.set_text(str(round(self.home_position_global_position_longitude[self.selected_UAV], 7)))


	def on_buttonHOMESYNC_clicked(self, button):
		home_latitude = self.home_position_global_position_latitude[1]
		home_longitude = self.home_position_global_position_longitude[1]
		home_altitude = self.home_position_global_position_altitude[1]
		msgHomePos = HomePosition()
		msgHomePos.geo.latitude = home_latitude
		msgHomePos.geo.longitude = home_longitude
		msgHomePos.geo.altitude = home_altitude
		self.home_point_pub[self.selected_UAV].publish(msgHomePos)

		self.lHP_LATITUDE.set_text(str(round(self.home_position_global_position_latitude[self.selected_UAV], 7)))
		self.lHP_LONGITUDE.set_text(str(round(self.home_position_global_position_longitude[self.selected_UAV], 7)))

#   /* LAND function */
	def on_buttonLAND_clicked(self, button):
		self.UAV_mode[0][self.selected_UAV] = 1
		resp = self.request_SetModeService[self.selected_UAV](0, "LAND")

	#/* TAKE-OFF handlers */
	def on_buttonTAKEOFF_clicked(self, button):

		if self.Z_cur_position[0][self.selected_UAV] < 0.1:	
			self.UAV_takeoff()

	#/* TAKE-OFF function */
	def UAV_takeoff(self):
		self.UAV_mode[0][self.selected_UAV] = 1
		resp = self.request_TakeOffService[self.selected_UAV](0,0,0,0,3)

#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#   /* POSITION HOLD handlers */
	def on_buttonHOLD_POS_clicked(self, button):	
		self.UAV_mode[0][self.selected_UAV] = 1

		if self.carrot_status[self.selected_UAV] == 1:
			desiredX = [0.0 for x in range(2)]
			desiredY = [0.0 for x in range(2)]
			desiredZ = [0.0 for x in range(2)]
			desiredW = [0.0 for x in range(2)]

			desiredX[0] = self.trajectory_ref_x_cur[self.selected_UAV]
			desiredY[0] = self.trajectory_ref_y_cur[self.selected_UAV]
			desiredZ[0] = self.trajectory_ref_z_cur[self.selected_UAV]+0.1
			desiredW[0] = self.trajectory_ref_w_cur[self.selected_UAV]
			desiredX[1] = self.trajectory_ref_x_cur[self.selected_UAV]
			desiredY[1] = self.trajectory_ref_y_cur[self.selected_UAV]
			desiredZ[1] = self.trajectory_ref_z_cur[self.selected_UAV]-0.1
			desiredW[1] = self.trajectory_ref_w_cur[self.selected_UAV]

			resp = self.request_TrajectoryFromGUIService(self.selected_UAV, self.UAV_mode[0][self.selected_UAV], desiredX, desiredY, desiredZ, desiredW, 1, 1, 0.5, 0.5)
			self.planned_trajectory[self.selected_UAV] = resp.multi_dof_trajectory_data

			msgMissionStart = MissionStartStop()
			msgMissionStart.selectedUAV = self.selected_UAV
			msgMissionStart.isPositionHoldActiveFlag = True
			msgMissionStart.isWPActiveFlag = False
			msgMissionStart.isBuildingTrajectoryActiveFlag = False
			msgMissionStart.multi_dof_trajectory_data = self.planned_trajectory[self.selected_UAV]
			self.mission_start_pub.publish(msgMissionStart)

	def on_buttonSEND_NEW_POS_clicked(self, button):
		self.UAV_mode[0][self.selected_UAV] = 1

		if self.carrot_status[self.selected_UAV] == 1:
			desiredX = [0.0 for x in range(2)]
			desiredY = [0.0 for x in range(2)]
			desiredZ = [0.0 for x in range(2)]
			desiredW = [0.0 for x in range(2)]

			desiredX[0] = self.trajectory_ref_x_cur[self.selected_UAV]
			desiredY[0] = self.trajectory_ref_y_cur[self.selected_UAV]
			desiredZ[0] = self.trajectory_ref_z_cur[self.selected_UAV]
			desiredW[0] = self.trajectory_ref_w_cur[self.selected_UAV]
			desiredX[1] = float(self.tbX.get_text())
			desiredY[1] = float(self.tbY.get_text())
			desiredZ[1] = float(self.tbZ.get_text())
			desiredW[1] = math.radians(float(self.tbW.get_text()))
			desiredHS = float(self.tbHS_PH.get_text())
			desiredVS = float(self.tbVS_PH.get_text())
			desiredHA = float(self.tbHA_PH.get_text())
			desiredVA = float(self.tbVA_PH.get_text())

			resp = self.request_TrajectoryFromGUIService(self.selected_UAV, self.UAV_mode[0][self.selected_UAV], desiredX, desiredY, desiredZ, desiredW, desiredHS, desiredVS, desiredHA, desiredVA)
			self.planned_trajectory[self.selected_UAV] = resp.multi_dof_trajectory_data

			msgMissionStart = MissionStartStop()
			msgMissionStart.selectedUAV = self.selected_UAV
			msgMissionStart.isPositionHoldActiveFlag = True
			msgMissionStart.isWPActiveFlag = False
			msgMissionStart.isBuildingTrajectoryActiveFlag = False
			msgMissionStart.multi_dof_trajectory_data = self.planned_trajectory[self.selected_UAV]
			self.mission_start_pub.publish(msgMissionStart)

			self.new_pos_counter[self.selected_UAV] = self.new_pos_counter[self.selected_UAV] + 1
			self.lPH_STAUTS.set_text(str(self.new_pos_counter[self.selected_UAV]) + ". new position sent to UAV " + self.selected_UAV_name)


	def on_buttonRETURN_HOME_clicked(self, button):	
		self.UAV_mode[0][self.selected_UAV] = 1

	#desired_tra_pub = rospy.Publisher(self.UAV_name+'_sim'+str(self.selected_UAV)+'/DesiredTrajectoryfromGUI', DesiredTrajectory, queue_size=10)

	#msgDesTra = DesiredTrajectory()
	#msgDesTra.desiredX = self.X_home_point[0][self.selected_UAV]
	#msgDesTra.desiredY = self.Y_home_point[0][self.selected_UAV]
	#msgDesTra.desiredZ = float(self.S_tbRTH_ALT.get_text())
	#msgDesTra.desiredW = self.W_cur_orientation[0][self.selected_UAV]
	#desired_tra_pub.publish(msgDesTra)

		desiredX = [0.0 for x in range(3)]
		desiredY = [0.0 for x in range(3)]
		desiredZ = [0.0 for x in range(3)]
		desiredW = [0.0 for x in range(3)]

		desiredX[0] = self.trajectory_ref_x_cur[self.selected_UAV]
		desiredY[0] = self.trajectory_ref_y_cur[self.selected_UAV]
		desiredZ[0] = self.trajectory_ref_z_cur[self.selected_UAV]
		desiredW[0] = self.trajectory_ref_w_cur[self.selected_UAV]
		desiredX[1] = self.trajectory_ref_x_cur[self.selected_UAV]
		desiredY[1] = self.trajectory_ref_y_cur[self.selected_UAV]
		desiredZ[1] = float(self.S_tbRTH_ALT.get_text())
		desiredW[1] = self.trajectory_ref_w_cur[self.selected_UAV]
		desiredX[2] = 0.0
		desiredY[2] = 0.0
		desiredZ[2] = float(self.S_tbRTH_ALT.get_text())
		desiredW[2] = self.trajectory_ref_w_cur[self.selected_UAV]

		resp = self.request_TrajectoryFromGUIService(self.selected_UAV, self.UAV_mode[0][self.selected_UAV], desiredX, desiredY, desiredZ, desiredW, 1, 1, 0.5, 0.5)
		self.planned_trajectory[self.selected_UAV] = resp.multi_dof_trajectory_data

		msgMissionStart = MissionStartStop()
		msgMissionStart.selectedUAV = self.selected_UAV
		msgMissionStart.isPositionHoldActiveFlag = True
		msgMissionStart.isWPActiveFlag = False
		msgMissionStart.isBuildingTrajectoryActiveFlag = False
		msgMissionStart.multi_dof_trajectory_data = self.planned_trajectory[self.selected_UAV]
		self.mission_start_pub.publish(msgMissionStart)

#   /* WAYPOINTS handlers */
	def on_buttonADD_WP_clicked(self, button):
		self.treeiter1 = self.liststore1.append([self.WP_num, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.WP_num = self.WP_num +1

	def on_buttonREMOVE_WP_clicked(self, button):
		selection = self.twWP.get_selection()
		model, paths = selection.get_selected_rows()
		for path in paths:
			iter = model.get_iter(path)
			model.remove(iter)

	def WP_edited_X(self, widget, path, text):
		self.liststore1[path][1] = float(text)

	def WP_edited_Y(self, widget, path, text):
		self.liststore1[path][2] = float(text)

	def WP_edited_Z(self, widget, path, text):
		self.liststore1[path][3] = float(text)

	def WP_edited_W(self, widget, path, text):
		self.liststore1[path][4] = float(text)

	def WP_edited_LT(self, widget, path, text):
		self.liststore1[path][5] = float(text)

	def on_radiobuttonWP_UAV_toggled(self, radiobutton, UAV_WP_selected):
		if radiobutton.get_active():
			if UAV_WP_selected == 1:
				self.UAV_WP = 1
			elif UAV_WP_selected == 2:
				self.UAV_WP = 2
			elif UAV_WP_selected == 3:
				self.UAV_WP = 3

	def on_buttonUPLOAD_WP_clicked(self, button):
		#mission_wp_pub = rospy.Publisher('MissionfromGUI', MissionWP, queue_size=10)

		msgMissionWP = MissionWP()
		msgMissionWP.selectedUAV = self.UAV_WP
		msgMissionWP.numberofWPs = len(self.liststore1)
		for row in self.liststore1:
			msgMissionWP.wpNumber.append(row[0])
			msgMissionWP.wpX.append(row[1])
			msgMissionWP.wpY.append(row[2])
			msgMissionWP.wpZ.append(row[3])
			msgMissionWP.wpW.append(row[4])
			msgMissionWP.wpLT.append(row[5])	
		self.mission_wp_pub.publish(msgMissionWP)

		print(msgMissionWP.numberofWPs)
		print(msgMissionWP.wpX)

	def on_buttonSTART_WP_clicked(self, button):
		self.UAV_mode[0][self.selected_UAV] = 2
		msgMissionStart = MissionStartStop()
		msgMissionStart.selectedUAV = self.selected_UAV
		msgMissionStart.isPositionHoldActiveFlag = False
		msgMissionStart.isWPActiveFlag = True
		msgMissionStart.isBuildingTrajectoryActiveFlag = False
		self.mission_start_pub.publish(msgMissionStart)

	def on_buttonWP_LOAD_MISSION_clicked(self, button):
		WP_param = ['0' for x in range(6)]

		dialog = Gtk.FileChooserDialog("Please choose a file", self.window, Gtk.FileChooserAction.OPEN, (Gtk.STOCK_OPEN, Gtk.ResponseType.OK, Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))

		response = dialog.run()
		if response == Gtk.ResponseType.OK:
 			WP_mission_file = dialog.get_file()
			isloaded, contents, etag = WP_mission_file.load_contents()
			i = 0
			j = 0
			contents = contents + '\0'
			while True:
				if (i == 0) and (j == 0):
					WP_param[j] = contents[i]
				else:
					if contents[i] == ',':
						j = j+1
						i = i+1
						WP_param[j] = contents[i]
					else:
						WP_param[j] = WP_param[j] + contents[i]
				i = i+1

				if contents[i] == '\n':
					#wp = int(WP_param[0])
		 			self.treeiter1 = self.liststore1.append([int(WP_param[0]), float(WP_param[1]), float(WP_param[2]), float(WP_param[3]), float(WP_param[4]), float(WP_param[5])])
					j = 0
					#i = i+1
					if contents[i+1] == '\0':
						break
					else:
						WP_param[j] = contents[i+1]
						i = i+2
			#rospy.loginfo(str(contents))	   
		elif response == Gtk.ResponseType.CANCEL:
			print("Cancel clicked")
		dialog.destroy()


#   /* POINT OF INTEREST handlers */
	def on_buttonPOISTART_clicked(self, button):
		self.UAV_mode[0][self.selected_UAV] = 3

		self.POI_param[0][self.selected_UAV][0] = float(self.tbPOIX.get_text())
		self.POI_param[0][self.selected_UAV][1] = float(self.tbPOIY.get_text())
		self.POI_param[0][self.selected_UAV][2] = float(self.tbPOIZ.get_text())

		self.POI_param[0][self.selected_UAV][3] = float(self.tbPOIR.get_text())
		self.POI_param[0][self.selected_UAV][4] = float(self.tbPOIA.get_text())
		self.POI_param[0][self.selected_UAV][5] = float(self.tbPOIV.get_text())

		self.POI_param[0][self.selected_UAV][6] = self.POI_direction

		self.POI_param[0][self.selected_UAV][7] = math.atan2((self.Y_cur_position[0][self.selected_UAV]-self.POI_param[0][self.selected_UAV][1]),(self.X_cur_position[0][self.selected_UAV]-self.POI_param[0][self.selected_UAV][0]))

		print(self.POI_param[0][self.selected_UAV][7])

		#self.POI_param[0][self.selected_UAV][7] = math.atan2(self.POI_param[0][self.selected_UAV][1],self.POI_param[0][self.selected_UAV][0])

		desiredX = [0.0 for x in range(2)]
		desiredY = [0.0 for x in range(2)]
		desiredZ = [0.0 for x in range(2)]
		desiredW = [0.0 for x in range(2)]

		desiredX[0] = self.trajectory_ref_x_cur[self.selected_UAV]
		desiredY[0] = self.trajectory_ref_y_cur[self.selected_UAV]
		desiredZ[0] = self.trajectory_ref_z_cur[self.selected_UAV]
		desiredW[0] = self.trajectory_ref_w_cur[self.selected_UAV]
		desiredX[1] = self.POI_param[0][self.selected_UAV][0] + self.POI_param[0][self.selected_UAV][3]*math.cos(self.POI_param[0][self.selected_UAV][7])
		desiredY[1] = self.POI_param[0][self.selected_UAV][1] + self.POI_param[0][self.selected_UAV][3]*math.sin(self.POI_param[0][self.selected_UAV][7])
		desiredZ[1] = self.POI_param[0][self.selected_UAV][2] + self.POI_param[0][self.selected_UAV][4]
		desiredW[1] = self.POI_param[0][self.selected_UAV][7] + math.pi

		self.POI_param[0][self.selected_UAV][7] = self.POI_param[0][self.selected_UAV][7] + (0.3*self.POI_param[0][self.selected_UAV][5]*self.POI_param[0][self.selected_UAV][6])/self.POI_param[0][self.selected_UAV][3]	# poiANG = poiANG + poiVEL*poiDIR/poiR



	#desired_tra_pub = rospy.Publisher(self.UAV_name+'_sim'+str(self.selected_UAV)+'/DesiredTrajectoryfromGUI', DesiredTrajectory, queue_size=10)
		#desired_tra_pub = rospy.Publisher('red/mavros/setpoint_position/local', PoseStamped, queue_size=10)



		#msgDesTra = PoseStamped()
		#msgDesTra.pose.position.x = self.POI_param[0][self.selected_UAV][0] + self.POI_param[0][self.selected_UAV][3]*math.cos(self.POI_param[0][self.selected_UAV][7]) 	# desired_X = poiX + poiR * cos(poiANG)
		#msgDesTra.pose.position.y = self.POI_param[0][self.selected_UAV][1] + self.POI_param[0][self.selected_UAV][3]*math.sin(self.POI_param[0][self.selected_UAV][7]) 	# desired_Y = poiY + poiR * sin(poiANG)
		#msgDesTra.pose.position.z = self.POI_param[0][self.selected_UAV][2] + self.POI_param[0][self.selected_UAV][4]

		#yaw = self.POI_param[0][self.selected_UAV][7] + (1*self.POI_param[0][self.selected_UAV][5]*self.POI_param[0][self.selected_UAV][6])/self.POI_param[0][self.selected_UAV][3]	# poiANG = poiANG + 0.01*poiVEL*poiDIR/poiR

		#cy = math.cos(yaw * 0.5)
		#sy = math.sin(yaw * 0.5)
		#cp = math.cos(0 * 0.5)
		#sp = math.sin(0 * 0.5)
		#cr = math.cos(0 * 0.5)
		#sr = math.sin(0 * 0.5)
		#msgDesTra.pose.orientation.x = sr * cp * cy - cr * sp * sy
		#msgDesTra.pose.orientation.y = cr * sp * cy + sr * cp * sy
		#msgDesTra.pose.orientation.z = cr * cp * sy - sr * sp * cy
		#msgDesTra.pose.orientation.w = cr * cp * cy + sr * sp * sy
		#desired_tra_pub.publish(msgDesTra)

		#msgDesTra = DesiredTrajectory()
		#msgDesTra.desiredX = self.POI_param[0][self.selected_UAV][0] + self.POI_param[0][self.selected_UAV][3]*math.cos(self.POI_param[0][self.selected_UAV][7]) 	# desired_X = poiX + poiR * cos(poiANG)
		#msgDesTra.desiredY = self.POI_param[0][self.selected_UAV][1] + self.POI_param[0][self.selected_UAV][3]*math.sin(self.POI_param[0][self.selected_UAV][7]) 	# desired_Y = poiY + poiR * sin(poiANG)
		#msgDesTra.desiredZ = self.POI_param[0][self.selected_UAV][2] + self.POI_param[0][self.selected_UAV][4]					# desired_Z = poiZ + poiALT
		#msgDesTra.desiredW = self.POI_param[0][self.selected_UAV][7]
			
		#desired_tra_pub.publish(msgDesTra)

		for n in range(0, int(2*self.POI_param[0][self.selected_UAV][3]*math.pi/0.1)):
		#for n in range(2, 90):
			#desiredX[n] = self.POI_param[0][self.selected_UAV][0] + self.POI_param[0][self.selected_UAV][3]*math.cos(self.POI_param[0][self.selected_UAV][7]) 		# desired_X = poiX + poiR * cos(poiANG)
			#desiredY[n] = self.POI_param[0][self.selected_UAV][1] + self.POI_param[0][self.selected_UAV][3]*math.sin(self.POI_param[0][self.selected_UAV][7]) 		# desired_Y = poiY + poiR * sin(poiANG)
			#desiredZ[n] = self.POI_param[0][self.selected_UAV][2] + self.POI_param[0][self.selected_UAV][4]														# desired_Z = poiZ + poiALT
			#desiredW[n] = self.POI_param[0][self.selected_UAV][7] + 3.14

			desiredX.append(self.POI_param[0][self.selected_UAV][0] + self.POI_param[0][self.selected_UAV][3]*math.cos(self.POI_param[0][self.selected_UAV][7])) 		# desired_X = poiX + poiR * cos(poiANG)
			desiredY.append(self.POI_param[0][self.selected_UAV][1] + self.POI_param[0][self.selected_UAV][3]*math.sin(self.POI_param[0][self.selected_UAV][7])) 		# desired_Y = poiY + poiR * sin(poiANG)
			desiredZ.append(self.POI_param[0][self.selected_UAV][2] + self.POI_param[0][self.selected_UAV][4])															# desired_Z = poiZ + poiALT
			desiredW.append(self.POI_param[0][self.selected_UAV][7] + math.pi)

			self.POI_param[0][self.selected_UAV][7] = self.POI_param[0][self.selected_UAV][7] + (0.1*self.POI_param[0][self.selected_UAV][6])/self.POI_param[0][self.selected_UAV][3]	# poiANG = poiANG + 0.1*poiDIR/poiR
			#self.POI_param[0][self.selected_UAV][7] = self.POI_param[0][self.selected_UAV][7] + 0.07*self.POI_param[0][self.selected_UAV][6]
			#print(self.POI_param[0][self.selected_UAV][7])
		
		plt.plot(desiredX, desiredY, 'g.')
		plt.plot(desiredX[1], desiredY[1], 'rx')
		plt.show()

		resp = self.request_TrajectoryFromGUIService(self.selected_UAV, self.UAV_mode[0][self.selected_UAV], desiredX, desiredY, desiredZ, desiredW, 1, 1, 0.5, 0.5)
		self.planned_trajectory[self.selected_UAV] = resp.multi_dof_trajectory_data

		mission_start_pub = rospy.Publisher('MissionStartStop', MissionStartStop, queue_size=10)
		msgMissionStart = MissionStartStop()
		msgMissionStart.selectedUAV = self.selected_UAV
		msgMissionStart.isPositionHoldActiveFlag = True
		msgMissionStart.isWPActiveFlag = False
		msgMissionStart.isBuildingTrajectoryActiveFlag = False
		msgMissionStart.multi_dof_trajectory_data = self.planned_trajectory[self.selected_UAV]
		mission_start_pub.publish(msgMissionStart)	


		self.POI_param[0][self.selected_UAV][8] = self.cur_odometry_time[0][self.selected_UAV]

	def on_radiobuttonPOICW_toggled(self, radiobutton):
		if radiobutton.get_active():
			self.POI_direction = -1
		else:
			self.POI_direction = 1


#   /* FOLLOW handlers */
	def on_buttonFOLLOW_clicked(self, button):
	
		if not [self.selected_UAV] == self.new_leader_UAV:
			self.UAV_mode[0][self.selected_UAV] = 4

		if self.follow_mode == 1:
			self.FOLLOW_param[self.selected_UAV][0] = math.hypot((self.X_cur_position[0][self.selected_UAV]-self.X_cur_position[0][self.new_leader_UAV]),(self.Y_cur_position[0][self.selected_UAV]-self.Y_cur_position[0][self.new_leader_UAV]))
			self.FOLLOW_param[self.selected_UAV][1] = math.atan2((self.Y_cur_position[0][self.selected_UAV]-self.Y_cur_position[0][self.new_leader_UAV]),(self.X_cur_position[0][self.selected_UAV]-self.X_cur_position[0][self.new_leader_UAV]))
			self.FOLLOW_param[self.selected_UAV][2] = self.Z_cur_position[0][self.selected_UAV] - self.Z_cur_position[0][self.new_leader_UAV]
		elif self.follow_mode == 2:
			self.FOLLOW_param[self.selected_UAV][0] = float(self.tbFdis.get_text())
			self.FOLLOW_param[self.selected_UAV][1] = float(self.tbFang.get_text())
			self.FOLLOW_param[self.selected_UAV][2] = float(self.tbFalt.get_text())

		self.leader_UAV[self.selected_UAV] = self.new_leader_UAV

	def on_radiobuttonUAV_toggled(self, radiobutton, leaderUAV):
		if radiobutton.get_active():
			if leaderUAV == 1:
				self.new_leader_UAV = 1
			elif leaderUAV == 2:
				self.new_leader_UAV = 2
			elif leaderUAV == 3:
				self.new_leader_UAV = 3

	def on_radiobuttonFMODE_toggled(self, radiobutton):
		if radiobutton.get_active():
			self.follow_mode = 1
		else:
			self.follow_mode = 2


#   /* TRAJECTORY PLANNER handlers */
	def on_buttonADD_BUILDING_POINT_clicked(self, button):
		self.treeiter4 = self.liststore4.append([self.BUILDING_POINT_num, 0.0, 0.0])
		self.BUILDING_POINT_num = self.BUILDING_POINT_num + 1

	def on_buttonREMOVE_BUILDING_POINT_clicked(self, button):
		selection = self.twTRAJ.get_selection()
		model, paths = selection.get_selected_rows()
		for path in paths:
   			iter = model.get_iter(path)
			model.remove(iter)

	def TRAJ_edited_X(self, widget, path, text):
		self.liststore4[path][1] = float(text)

	def TRAJ_edited_Y(self, widget, path, text):
		self.liststore4[path][2] = float(text)

	def on_buttonLOAD_BUILDING_FILE_clicked(self, button):
		BUILDING_param = ['0' for x in range(3)]

		dialog = Gtk.FileChooserDialog("Please choose a file", self.window, Gtk.FileChooserAction.OPEN, (Gtk.STOCK_OPEN, Gtk.ResponseType.OK, Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))

		response = dialog.run()
	   	if response == Gtk.ResponseType.OK:
 			BUILDING_file = dialog.get_file()
			isloaded, contents, etag = BUILDING_file.load_contents()
			i = 0
			j = 0
			contents = contents + '\0'
			while True:
				if (i == 0) and (j == 0):
					BUILDING_param[j] = contents[i]
				else:
					if contents[i] == ',':
						j = j+1
						i = i+1
						BUILDING_param[j] = contents[i]
					else:
						BUILDING_param[j] = BUILDING_param[j] + contents[i]
				i = i+1

				if contents[i] == '\n':
					#wp = int(WP_param[0])
			 		self.treeiter4 = self.liststore4.append([int(BUILDING_param[0]), float(BUILDING_param[1]), float(BUILDING_param[2])])
					j = 0
					#i = i+1
					if contents[i+1] == '\0':
						break
					else:
						BUILDING_param[j] = contents[i+1]
						i = i+2
		#rospy.loginfo(str(contents))	   
		elif response == Gtk.ResponseType.CANCEL:
			print("Cancel clicked")
		dialog.destroy()

	def on_buttonPLAN_TRAJECTORY_clicked(self, button):
		building_x = []
		building_y = []
		flight_altitude = float(self.tbTRAJ_ALT.get_text())
		building_distance = float(self.tbBUIL_DIST.get_text())
		trajectory_resolution = float(self.tbWP_RES.get_text())
		wp_file_path = self.tbWP_PATH.get_text()
		coord_sys_text = self.cbtCOORDSYS.get_active_text()
		h_speed = float(self.tbHS_TP.get_text())
		v_speed = float(self.tbVS_TP.get_text())
		h_acc = float(self.tbHA_TP.get_text())
		v_acc = float(self.tbVA_TP.get_text())

		for row in self.liststore4:
			building_x.append(row[1])
			building_y.append(row[2])

		resp = self.request_TrajectoryPlanningService(building_x, building_y, flight_altitude, building_distance, trajectory_resolution, wp_file_path, coord_sys_text, h_speed, v_speed, h_acc, v_acc)
		self.planned_trajectory[self.selected_UAV] = resp.multi_dof_trajectory_data
		self.isBuildingTrajectoryReady[self.selected_UAV] = True

	#print(self.planned_trajectory[self.selected_UAV].points[8])
	
	def on_buttonEXECUTE_TRAJECTORY_clicked(self, button):
		self.UAV_mode[0][self.selected_UAV] = 5

		if self.isBuildingTrajectoryReady[self.selected_UAV]:
			#mission_start_pub = rospy.Publisher('MissionStartStop', MissionStartStop, queue_size=10)
			msgMissionStart = MissionStartStop()
			msgMissionStart.selectedUAV = self.selected_UAV
			msgMissionStart.isPositionHoldActiveFlag = False
			msgMissionStart.isWPActiveFlag = False
			msgMissionStart.isBuildingTrajectoryActiveFlag = True
			msgMissionStart.multi_dof_trajectory_data = self.planned_trajectory[self.selected_UAV]
			self.mission_start_pub.publish(msgMissionStart)

			self.isBuildingTrajectoryReady[self.selected_UAV] = False
	

#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#   /* BATTERY SIM handlers */
	def BatteryCallback(self, data):

		for n in range(1, self.number_of_UAVs+1):
			self.max_batt_data[0][n] = data.maxBatt[n]
			self.new_batt_data[0][n] = data.curBatt[n]

#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#   /* UAV STATUS updadtes */
	def GUI_status_update_ALL(self):

		for n in range(1, self.number_of_UAVs+1):	
 			self.lbTELEM_X[n].set_text(str(round(self.X_cur_position[0][n], 2)))
 			self.lbTELEM_Y[n].set_text(str(round(self.Y_cur_position[0][n], 2)))
 			self.lbTELEM_Z[n].set_text(str(round(self.Z_cur_position[0][n], 2)))
 			self.lbTELEM_HEADING[n].set_text(str(round(self.W_cur_orientation[0][n], 2)))
			self.lbTELEM_HS[n].set_text(str(round(self.H_speed[0][n], 2)))
			self.lbTELEM_VS[n].set_text(str(round(self.V_speed[0][n], 2)))
			self.pbTELEM_BATTERY[n].set_fraction(self.UAV_batt_data[0][n])


	def GUI_status_update_SELECTED_UAV(self):

		if self.UAV_state_mode[self.selected_UAV] == 0:
			self.lMODE.set_text("LAND")
			#self.lAM.set_text("LAND")
		elif self.UAV_state_mode[self.selected_UAV] == 1:
			self.lMODE.set_text("ALT HOLD")
			#self.lAM.set_text("ALT HOLD")
		elif self.UAV_state_mode[self.selected_UAV] == 2:
				self.lMODE.set_text("GUIDED")
			#self.lAM.set_text("GUIDED")
		elif self.UAV_state_mode[self.selected_UAV] == 3:
			self.lMODE.set_text("LOITER")
			#self.lAM.set_text("LOITER")
		elif self.UAV_state_mode[self.selected_UAV] == 4:
			self.lMODE.set_text("GUIDED_NOGPS")

		if self.UAV_state_armed[self.selected_UAV] == True:
			self.lARMED.set_text("ARMED")
		else:
			self.lARMED.set_text("DISARMED")

		if self.carrot_status[self.selected_UAV] == 0:
			self.lCARROTSTATUS.set_text("OFF")
			#self.swCARROT_CONTOLLER.set_active(False)
				#self.lAM.set_text("LAND")
		elif self.carrot_status[self.selected_UAV] == 1:
			self.lCARROTSTATUS.set_text("HOLD")
			#self.lAM.set_text("ALT HOLD")
		elif self.carrot_status[self.selected_UAV] == 2:
			self.lCARROTSTATUS.set_text("CARROT_ON_AIR")

		selectedUAV_pub = rospy.Publisher("SelectedUAVonGraupner", SelectedUAV, queue_size=10)
		msgSelectedUAV = SelectedUAV()
		msgSelectedUAV.selectedUAV = self.selected_UAV
		msgSelectedUAV.selectedUAV_mode = self.UAV_mode[0][self.selected_UAV]
			
		selectedUAV_pub.publish(msgSelectedUAV)

		#self.S_lGPSLOCK.set_text(str(self.CH5))
		#self.S_lGPSLOCK.set_angle(math.degrees(self.W_cur_orientation[0][self.selected_UAV]))
		self.lLOCALX.set_text(str(round(self.X_cur_position[0][self.selected_UAV], 2)))
		self.lLOCALY.set_text(str(round(self.Y_cur_position[0][self.selected_UAV], 2)))
		self.lLOCALZ.set_text(str(round(self.Z_cur_position[0][self.selected_UAV], 2)))
		self.lLATITUDE.set_text(str(round(self.lat_cur_global[self.selected_UAV], 7)))
		self.lLONGITUDE.set_text(str(round(self.long_cur_global[self.selected_UAV], 7)))
		self.lALTITUDE.set_text(str(round(self.alt_cur_global[self.selected_UAV], 3)))
		self.lHEADING.set_text(str(round(math.degrees(self.W_cur_orientation[0][self.selected_UAV]),)))
		self.lHS.set_text(str(round(self.H_speed[0][self.selected_UAV], 2)))
		self.lVS.set_text(str(round(self.V_speed[0][self.selected_UAV], 2)))





		if self.nb1.get_current_page() == 0:
			self.lX.set_text(str(round(self.X_cur_position[0][self.selected_UAV], 2)))
			self.lY.set_text(str(round(self.Y_cur_position[0][self.selected_UAV], 2)))
			self.lZ.set_text(str(round(self.Z_cur_position[0][self.selected_UAV], 2)))
			self.lW.set_text(str(round(math.degrees(self.W_cur_orientation[0][self.selected_UAV]), )))
		if self.nb1.get_current_page() == 3:
			if not self.selected_UAV == self.new_leader_UAV:
				self.lFdis.set_text(str(round(math.hypot((self.X_cur_position[0][self.selected_UAV]-self.X_cur_position[0][self.new_leader_UAV]),(self.Y_cur_position[0][self.selected_UAV]-self.Y_cur_position[0][self.new_leader_UAV])), 2)))
				self.lFang.set_text(str(round(math.atan2((self.Y_cur_position[0][self.selected_UAV]-self.Y_cur_position[0][self.new_leader_UAV]),(self.X_cur_position[0][self.selected_UAV]-self.X_cur_position[0][self.new_leader_UAV])), 2)))
				self.lFalt.set_text(str(round(self.Z_cur_position[0][self.selected_UAV]-self.Z_cur_position[0][self.new_leader_UAV], 2)))
			else:
				self.lFdis.set_text("0.00")
				self.lFang.set_text("0.00")
				self.lFalt.set_text("0.00")
		if self.nb1.get_current_page() == 4:
			if self.isBuildingTrajectoryReady[self.selected_UAV]:
				self.S_lTRAJSTATUS.set_text("Trajectory ready to execute")
			else:
				self.S_lTRAJSTATUS.set_text("Trajectory not ready")


		#if (cur_odometry_time[0][selected_UAV] - old_odometry_time[0][selected_UAV]) > 0.01:
		self.window.queue_draw_area(0,0,1000,1000)
		self.S_daHEADING.connect("draw", self.draw_S_HEADING_background_update)
		self.S_daHEADING.connect("draw", self.draw_S_HEADING_update)



	def GUI_battery_update(self):

		for n in range(1, self.number_of_UAVs+1):
			self.UAV_batt_data[0][n] = float(self.new_batt_data[0][n])/self.max_batt_data[0][n]
			#lbTELEM_BATTERY[n].set_text(str(UAV_batt_data[0][n]))
			#liststore3[Gtk.TreePath(n-1)][0] = new_batt_data[n]

		self.pb1.set_fraction(self.UAV_batt_data[0][self.selected_UAV])
		self.lBATT.set_text("UAV "+str(self.selected_UAV)+" BATTERY")



	def draw_S_HEADING_update(self, wid, cr):

		rotation_angle = self.W_cur_orientation[0][self.selected_UAV]

		cr.translate(160*0.5, 160*0.5)
		cr.rotate(-rotation_angle)
		cr.translate(-160*0.5, -160*0.5)
		cr.scale(1.0, 1.0)

		cr.set_source_surface(self.arrow_surface_1, 30, 30)
		cr.paint()

		cr.translate(160*0.5, 160*0.5)
		cr.rotate(rotation_angle)
		cr.translate(-160*0.5, -160*0.5)
		cr.scale(1.0, 1.0)



	def draw_S_HEADING_background_update(self, wid, cr):
	
		cr.set_source_surface(self.arrow_surface_blank_1, 0, 0)
		cr.paint()

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

#def ImageCallback(data):
#
#	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#	cv2.imwrite("bebop_camera.png", cv_image)
#	im1.set_from_resource("bebop_camera.png")


#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------------------------------------------------------------------------#
	def run(self):

		self.RCController_status = self.cbtRCCONTROLLER.get_active_text()

		if self.RCController_status == 'GraupnerRC':
			for stick in self.sticks:
				stick.update_stick(self.device, self.CH1, self.CH2, self.CH3, self.CH4, self.CH5, self.CH6)
		elif (self.RCController_status == 'N/A') and (self.carrot_status[self.selected_UAV] == 2):
			self.CH1 = self.scTHROTTLE.get_value()
			self.CH2 = self.scROLL.get_value()
			self.CH3 = self.scPITCH.get_value()
			self.CH4 = self.scYAW.get_value()
			scaled_CH1 = (float(self.CH1 - 1500))/400
			scaled_CH2 = (float(-self.CH2 + 1500))/400
			scaled_CH3 = (float(self.CH3 - 1500))/400
			scaled_CH4 = (float(self.CH4 - 1500))/400
			#simRC_pub = rospy.Publisher(self.selected_UAV_name+'/joy', Joy, queue_size=10)
			msgSimRC = Joy()
			msgSimRC.axes = [scaled_CH4, scaled_CH1, scaled_CH2, scaled_CH3, self.CH5, self.CH6, 0, 0]
			msgSimRC.buttons = [0,0,0,0,0,1]
			self.simRC_pub[self.selected_UAV].publish(msgSimRC)

		self.lbS1.set_value(self.CH1)
		if self.CH4 < 1500:
			self.lbS2_1.set_value(1500-abs(self.CH4-1100))
			self.lbS2_2.set_value(1500)
		elif self.CH4 > 1500:
			self.lbS2_1.set_value(1100)
			self.lbS2_2.set_value(self.CH4)
		else:
			self.lbS2_1.set_value(1100)
			self.lbS2_2.set_value(1500)

		if self.CH3 > 1500:
			self.lbS3_1.set_value(self.CH3)
			self.lbS3_2.set_value(1100)
		elif self.CH3 < 1500:
			self.lbS3_1.set_value(1500)
			self.lbS3_2.set_value(1500-abs(self.CH3-1100))
		else:
			self.lbS3_1.set_value(1500)
			self.lbS3_2.set_value(1100)

		if self.CH2 > 1500:
			self.lbS4_1.set_value(self.CH2)
			self.lbS4_2.set_value(1100)
		elif self.CH2 < 1500:
			self.lbS4_1.set_value(1500)
			self.lbS4_2.set_value(1500-abs(self.CH2-1100))
		else:
			self.lbS4_1.set_value(1500)
			self.lbS4_2.set_value(1100)

		if self.CH5 < 1200:
			self.selected_UAV = 1
			self.selected_UAV_name = self.UAV_names[1]
			self.lb5.set_value(1)
			self.lb6.set_value(0)
			self.lb7.set_value(0)
		elif (self.CH5 > 1200) and (self.CH5 < 1800):
			self.selected_UAV = 2
			self.selected_UAV_name = self.UAV_names[2]
			self.lb5.set_value(0)
			self.lb6.set_value(1)
			self.lb7.set_value(0)
		elif self.CH5 > 1800:
			self.selected_UAV = 3
			self.selected_UAV_name = self.UAV_names[3]
			self.lb5.set_value(0)
			self.lb6.set_value(0)
			self.lb7.set_value(1)

	#if self.CH6 < 1500:
	#	self.lb8.set_value(0)
	#	self.lb9.set_value(1)
		#lb10.set_value(1)
		#lb11.set_value(0)
	#else:
	#	self.lb8.set_value(1)
	#	self.lb9.set_value(0)
		#mode_flags[0][selected_UAV][0] = True
	#	self.UAV_mode[0][self.selected_UAV] = 0
		#lb10.set_value(0)
		#lb11.set_value(1)

		self.GUI_status_update_ALL()
		self.GUI_battery_update()
		self.GUI_status_update_SELECTED_UAV()

	#rospy.loginfo(str(self.CH5))

		for n in range(1, self.number_of_UAVs+1):
			if self.UAV_mode[0][n] == 0:
				self.lbTELEM_MODE[n].set_text("manual")
			if self.UAV_mode[0][n] == 1:
				self.lbTELEM_MODE[n].set_text("hold pos")
			if self.UAV_mode[0][n] == 2:
				self.lbTELEM_MODE[n].set_text("wp mission")
			if self.UAV_mode[0][n] == 3:
				self.lbTELEM_MODE[n].set_text("POI")
			if self.UAV_mode[0][n] == 4:
				self.lbTELEM_MODE[n].set_text("follow")
			if self.UAV_mode[0][n] == 5:
				self.lbTELEM_MODE[n].set_text("trajectory")


	#/* POINT OF INTEREST MODE */




		#/* FOLLOW MODE */

		for n in range(1, self.number_of_UAVs+1):
#			if (mode_flags[0][n][4]):
			if self.UAV_mode[0][n] == 4:

				desired_tra_pub = rospy.Publisher(self.UAV_name+'_sim'+str(n)+'/DesiredTrajectoryfromGUI', DesiredTrajectory, queue_size=10)

				msgDesTra = DesiredTrajectory()
				msgDesTra.desiredX = self.X_cur_position[0][self.leader_UAV[n]] + self.FOLLOW_param[n][0]*math.cos(self.FOLLOW_param[n][1])	# desired_X = leader_X + follow_distance*cos(follow_angle)
				msgDesTra.desiredY = self.Y_cur_position[0][self.leader_UAV[n]] + self.FOLLOW_param[n][0]*math.sin(self.FOLLOW_param[n][1])	# desired_Y = leader_Y + follow_distance*sin(follow_angle)
				msgDesTra.desiredZ = self.Z_cur_position[0][self.leader_UAV[n]] + self.FOLLOW_param[n][2]					# desired_Z = leader_Z + follow_altitude
				msgDesTra.desiredW = self.W_cur_orientation[0][n]

				desired_tra_pub.publish(msgDesTra)

		#/* TRAJECTORY MODE */


		return 1

#class GUIappGTK(threading.Thread):
	
	#def run(self):
	   #Gtk.main()

#___________________________________________________


if __name__ == "__main__":
	GUIapplication = app_main()
	#GUIapplicationROS = app_main()
	#GUIapplicationGTK = GUIappGTK()
	#GUIapplicationROS.daemon = True
	#GUIapplicationROS.start()
	#GUIapplicationGTK.start()
	GLib.timeout_add(100, GUIapplication.run)
	Gtk.main()
	#while not rospy.is_shutdown():
	#GLib.idle_add(GUIapplicationROS.run)
	#time.sleep(0.1)

