#!/usr/bin/env python

import copy, time

# Ros imports
import rospy, math

from math import sin, cos

from nav_msgs.msg import Odometry

from graupner_serial.msg import MissionWP, MissionStartStop

from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest

from geographic_msgs.msg import GeoPoint	

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
	MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
from mavros_msgs.msg import HomePosition

from graupner_serial.msg import SelectedUAV

from graupner_serial.srv import BuildingDescriptor, BuildingTrajectoryParameters_Carrot, LatLong
from math import atan2, pi
import matplotlib.pyplot as plt
from math import sin, cos


class RequestTrajectory():

	def __init__(self):

		self.number_of_UAVs = 3
		self.selected_UAV = 0
		self.position = [[0.0 for x in range(self.number_of_UAVs+1)] for y in range(self.number_of_UAVs+1)]
		for i in range(self.number_of_UAVs+1):
			self.position[i][2] = 1.0

		self.UAV_names = ["UAV", "red", "blue", "yellow"]
		self.home_position_global_position = [GeoPoint for x in range(self.number_of_UAVs+1)]

		self.trajectory_ref_x_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_y_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_z_cur = [0.0 for x in range(self.number_of_UAVs+1)]
		self.trajectory_ref_w_cur = [0.0 for x in range(self.number_of_UAVs+1)]



		# First set up service
		self.service = rospy.Service('TrajectoryPlanningService', BuildingTrajectoryParameters_Carrot, self.trajectory_setup)

		self.request_geolib_service = rospy.ServiceProxy("geographiclib_global2local_service", LatLong)

		self.request_generate_trajectoy_service = rospy.ServiceProxy("generate_trajectory_points_service", BuildingDescriptor)

		self.request_trajectory_service = rospy.ServiceProxy("generate_toppra_trajectory", GenerateTrajectory)

		# This is an example for UAV and output trajectory is converted 
		# accordingly
		#trajectory_pub = rospy.Publisher('/building_envelope/joint_trajectory', 
		#	JointTrajectory, queue_size=100)

		#home_pub = rospy.Publisher('/mavros/global_position/home', 
		#	HomePosition, queue_size=1)
		
		#self.position = [0.0, 0.0, 1.0, 0.0]

	   
	#uav_pose_sub = rospy.Subscriber('/red/carrot/pose',
		#	PoseStamped, pose_update)

	#self.UAV_name = rospy.get_param('uav_name_param')
		for n in range(1, self.number_of_UAVs+1):
			rospy.Subscriber(self.UAV_names[n]+"/mavros/global_position/local", Odometry, self.OdometryCallback, callback_args=n)
			#rospy.Subscriber(self.UAV_names[n]+"/mavros/home_position/home", HomePosition, self.HomePositionCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/mavros/global_position/home", HomePosition, self.HomePositionCallback, callback_args=n)
			rospy.Subscriber(self.UAV_names[n]+"/carrot/trajectory", MultiDOFJointTrajectoryPoint, self.CarrotTrajectoryCallback, callback_args=n)
		
		rospy.Subscriber("SelectedUAVonGraupner", SelectedUAV, self.SelectedUAVCallback)

		time.sleep(0.5)

	def trajectory_setup(self, req):

		x = req.building_x
		y = req.building_y
		z = [1 for i in range(len(x))]
		yaw = [0 for i in range(len(x))]
		height = req.flight_altitude
		dist = req.building_distance
		res = req.trajectory_resolution
		res_building_points = 0.1
		res_height = 100
		wp_file_path = req.wp_file_path
		coord_sys_text = req.coord_sys_text
		traj_wp_HS = req.h_speed
		traj_wp_VS = req.v_speed
		traj_wp_HA = req.h_acc
		traj_wp_VA = req.v_acc

		#latitude = [45.807742, 45.807643, 45.808806, 45.808866]	# mimara (u 4 tocke)
		#longitude = [15.966094, 15.967185, 15.967314, 15.966224]
		#latitude = [45.807742, 45.807733, 45.807794, 45.807800]	# mali cetverokut (7x7 m)
		#longitude = [15.966094, 15.966196, 15.966208, 15.966114]
		if coord_sys_text == 'WGS84':
			longitude = x
			latitude = y
			#resp = self.request_geolib_service(latitude, longitude, latitude[0], longitude[0])
			#resp = self.request_geolib_service(latitude, longitude, 45.813988, 16.038427)
			#resp = self.request_geolib_service(latitude, longitude, 45.813993, 16.038573)
			self.home_position_global_position[self.selected_UAV].latitude = 45.813988
			self.home_position_global_position[self.selected_UAV].longitude = 16.038427
			self.home_position_global_position[self.selected_UAV].altitude = 120
			print(self.home_position_global_position[self.selected_UAV].latitude)
			print(self.home_position_global_position[self.selected_UAV].longitude)
			#resp = self.request_geolib_service(latitude, longitude, self.home_position_global_position[self.selected_UAV].latitude, self.home_position_global_position[self.selected_UAV].longitude)
			resp = self.request_geolib_service(latitude, longitude, self.home_position_global_position[self.selected_UAV].latitude, self.home_position_global_position[self.selected_UAV].longitude, self.home_position_global_position[self.selected_UAV].altitude)
			x = resp.x
			y = resp.y
			plotx = []; ploty = []
			plotx = list(x)
			ploty = list(y)

		resp = self.request_generate_trajectoy_service(x, y, height, dist, res, res_building_points, res_height)
		traj_wp_x = [self.trajectory_ref_x_cur[self.selected_UAV]] + [(self.trajectory_ref_x_cur[self.selected_UAV] + resp.traj_x[0])/2] + list(resp.traj_x)
		traj_wp_y = [self.trajectory_ref_y_cur[self.selected_UAV]] + [(self.trajectory_ref_y_cur[self.selected_UAV] + resp.traj_y[0])/2] + list(resp.traj_y)
		traj_wp_z = [self.trajectory_ref_z_cur[self.selected_UAV]] + [(self.trajectory_ref_z_cur[self.selected_UAV] + resp.traj_z[0])/2] + list(resp.traj_z)
		traj_wp_yaw = [self.trajectory_ref_w_cur[self.selected_UAV]] + [resp.traj_yaw[0]] + list(resp.traj_yaw)
		print(resp.traj_x[0])

		#traj_wp_x = [self.trajectory_ref_x_cur[self.selected_UAV]] + list(resp.traj_x)
		#traj_wp_y = [self.trajectory_ref_y_cur[self.selected_UAV]] + list(resp.traj_y)
		#traj_wp_z = [self.trajectory_ref_z_cur[self.selected_UAV]] + list(resp.traj_z)
		#traj_wp_yaw = [self.trajectory_ref_w_cur[self.selected_UAV]] + list(resp.traj_yaw)

		#traj_wp_x = list(resp.traj_x)
		#traj_wp_y = list(resp.traj_y)
		#traj_wp_z = list(resp.traj_z)
		#traj_wp_yaw = list(resp.traj_yaw)
		#plotx = []; ploty = []
		#for i in range(len(traj_wp_yaw)):
		#	plotx.append(i)
		#	ploty.append(traj_wp_yaw[i])
		#plt.plot(plotx, ploty, 'g.')
		#plt.show()



		traj_wp_yaw = self.fix_angle_discontinuities(traj_wp_yaw)
		
		with open(wp_file_path, 'w') as f:
			for i in range(len(traj_wp_x)):
				f.write("%d,%f,%f,%f,%f\n" % (i+1, traj_wp_x[i], traj_wp_y[i], traj_wp_z[i], traj_wp_yaw[i]))

	#/_______________________________________________________________

	#mission_wp_pub = rospy.Publisher('MissionfromGUI', MissionWP, queue_size=10)
	#msgMissionWP = MissionWP()
	#msgMissionWP.selectedUAV = self.selected_UAV
	#msgMissionWP.numberofWPs = len(traj_wp_x)
		#for i in range(0, len(traj_wp_x)):
	#	msgMissionWP.wpNumber.append(i)
	#	msgMissionWP.wpX.append(traj_wp_x[i])
	#	msgMissionWP.wpY.append(traj_wp_y[i])
	#	msgMissionWP.wpZ.append(traj_wp_z[i])
	#	msgMissionWP.wpW.append(traj_wp_yaw[i])
	#	msgMissionWP.wpLT.append(0)	
	#mission_wp_pub.publish(msgMissionWP)


	#mission_start_pub = rospy.Publisher('MissionStartStop', MissionStartStop, queue_size=10)
	#msgMissionStart = MissionStartStop()
	#msgMissionStart.selectedUAV = self.selected_UAV
	#msgMissionStart.isActiveFlag = True
	#mission_start_pub.publish(msgMissionStart)

	#/__________________________________________________________________________________________

		#plotx = []; ploty = []
		#for i in range(len(traj_wp_yaw)):
		#	plotx.append(traj_wp_x[i] + cos(traj_wp_yaw[i]))
		#	ploty.append(traj_wp_y[i] + sin(traj_wp_yaw[i]))
		#plt.plot(plotx, ploty, 'r.')
		#plt.show()

		#plotx = []; ploty = []
		#for i in range(len(traj_wp_yaw)):
		#	plotx.append(i)
		#	ploty.append(traj_wp_yaw[i])
		#plt.plot(plotx, ploty, 'r.')
		#plt.show()
	
		# Create a service request which will be filled with waypoints
		request = GenerateTrajectoryRequest()

		# Add waypoints in request
		waypoint = JointTrajectoryPoint()
		for i in range(0, len(traj_wp_x)):
			# Positions are defined above
			waypoint.positions = [traj_wp_x[i], traj_wp_y[i], traj_wp_z[i], traj_wp_yaw[i]]
			# Also add constraints for velocity and acceleration. These
			# constraints are added only on the first waypoint since the
			# TOPP-RA reads them only from there.
			if i==0:
				waypoint.velocities = [traj_wp_HS, traj_wp_HS, traj_wp_VS, 1]
				waypoint.accelerations = [traj_wp_HA, traj_wp_HA, traj_wp_VA, 0.3]

			# Append all waypoints in request
			request.waypoints.points.append(copy.deepcopy(waypoint))

		# Set up joint names. This step is not necessary
		request.waypoints.joint_names = ["x", "y", "z", "yaw"]
		# Set up sampling frequency of output trajectory.
		request.sampling_frequency = 100.0
		# Set up number of gridpoints. The more gridpoints there are, 
		# trajectory interpolation will be more accurate but slower.
		# Defaults to 100
		request.n_gridpoints = 100
		# If you want to plot Maximum Velocity Curve and accelerations you can
		# send True in this field. This is intended to be used only when you
		# have to debug something since it will block the service until plot
		# is closed.
		request.plot = False
		# Request the trajectory
		response = self.request_trajectory_service(request)

		#plotx = []
		#ploty = []
		for i in range(len(response.trajectory.points)):
			plotx.append(response.trajectory.points[i].positions[0])
			ploty.append(response.trajectory.points[i].positions[1])
		plt.plot(plotx,ploty, 'b.')
		plt.show()

		# Response will have trajectory and bool variable success. If for some
		# reason the trajectory was not able to be planned or the configuration
		# was incomplete or wrong it will return False.

		print("Converting trajectory to multi dof")
		joint_trajectory = response.trajectory
		multi_dof_trajectory_data = MultiDOFJointTrajectory()
		multi_dof_trajectory_data = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
	#print(multi_dof_trajectory_data)
		print("Sending trajectory")
		#trajectory_pub.publish(joint_trajectory)
		trajectory_ready_status = True
	#Publishing multi_dof_trajectory
	#multi_dof_traj_pub = rospy.Publisher(self.UAV_name+'_sim'+str(self.selected_UAV)+'/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

		return multi_dof_trajectory_data		

	def JointTrajectory2MultiDofTrajectory(self, joint_trajectory):
		multi_dof_trajectory = MultiDOFJointTrajectory()

		for i in range(0, len(joint_trajectory.points)):
			temp_point = MultiDOFJointTrajectoryPoint()
			temp_transform = Transform()
			temp_transform.translation.x = joint_trajectory.points[i].positions[0]
			temp_transform.translation.y = joint_trajectory.points[i].positions[1]
			temp_transform.translation.z = joint_trajectory.points[i].positions[2]
			temp_transform.rotation.z = math.sin(joint_trajectory.points[i].positions[3]/2.0)
			temp_transform.rotation.w = math.cos(joint_trajectory.points[i].positions[3]/2.0)

			temp_vel = Twist()
			temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
			temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
			temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

			temp_acc = Twist()
			temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
			temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
			temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

			temp_point.transforms.append(temp_transform)
			temp_point.velocities.append(temp_vel)
			temp_point.accelerations.append(temp_acc)
			temp_point.time_from_start = joint_trajectory.points[i].time_from_start

		#multi_dof_traj_pub = rospy.Publisher('red/carrot/trajectory', MultiDOFJointTrajectoryPoint, queue_size=10)
		#multi_dof_traj_pub.publish(temp_point)

			multi_dof_trajectory.points.append(temp_point)
		#time.sleep(0.01)

		return multi_dof_trajectory
	

	def fix_angle_discontinuities(self, yaw_list):
		for i in range(1,len(yaw_list)):
			delta = yaw_list[i] - yaw_list[i-1]
			if delta > math.pi:
				for k in range(i,len(yaw_list)):
					yaw_list[k] = yaw_list[k] - 2*math.pi
			elif delta < -math.pi:
				for k in range(i,len(yaw_list)):
					yaw_list[k] = yaw_list[k] + 2*math.pi
		return yaw_list


	def OdometryCallback(self, data, UAV_ID):
		self.position[UAV_ID][0] = data.pose.pose.position.x
		self.position[UAV_ID][1] = data.pose.pose.position.y
		self.position[UAV_ID][2] = data.pose.pose.position.z
		q = data.pose.pose.orientation
		self.position[UAV_ID][3] = atan2( 2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z )

	def HomePositionCallback(self, data, UAV_ID):
		self.home_position_global_position[UAV_ID] = data.geo
		#self.home_position_local_position[UAV_ID] = data.position
		#self.home_position_local_orientation[UAV_ID] = data.orientation

	def CarrotTrajectoryCallback(self, data, UAV_ID):
		self.trajectory_ref_x_cur[UAV_ID] = data.transforms[0].translation.x
		self.trajectory_ref_y_cur[UAV_ID] = data.transforms[0].translation.y
		self.trajectory_ref_z_cur[UAV_ID] = data.transforms[0].translation.z
		self.trajectory_ref_w_cur[UAV_ID] = math.atan2(2.0 * (data.transforms[0].rotation.w * data.transforms[0].rotation.z + data.transforms[0].rotation.x * data.transforms[0].rotation.y), 1.0 - 2.0 * (data.transforms[0].rotation.y * data.transforms[0].rotation.y + data.transforms[0].rotation.z * data.transforms[0].rotation.z))



	def SelectedUAVCallback(self, data):
		self.selected_UAV = data.selectedUAV
		#self.selected_UAV_mode = data.selectedUAV_mode

	
if __name__ == "__main__":
	rospy.init_node("TrajectoryPlanningServiceNode")
	RequestTrajectory()
	rospy.spin()
