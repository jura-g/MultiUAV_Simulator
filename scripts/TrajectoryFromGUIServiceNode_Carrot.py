#!/usr/bin/env python

import copy, time

# Ros imports
import rospy, math

from math import sin, cos

from nav_msgs.msg import Odometry

from graupner_serial.msg import MissionWP, MissionStartStop

from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
	MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
from mavros_msgs.msg import HomePosition

from graupner_serial.msg import SelectedUAV

from graupner_serial.srv import BuildingDescriptor, GUITrajectoryParameters_Carrot, LatLong
from math import atan2, pi
import matplotlib.pyplot as plt
from math import sin, cos


class RequestTrajectory():

	def __init__(self):

		self.number_of_UAVs = 3
		self.selected_UAV = 0
		self.selected_UAV_mode = 0
		self.position = [[0.0 for x in range(self.number_of_UAVs+1)] for y in range(4)]
		for i in range(self.number_of_UAVs+1):
			self.position[i][2] = 1.0

		# First set up service
		self.service = rospy.Service('TrajectoryFromGUIService', GUITrajectoryParameters_Carrot, self.trajectory_setup)

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
		rospy.Subscriber("red/mavros/global_position/local", Odometry, self.OdometryCallback, callback_args=1)
		rospy.Subscriber("blue/mavros/global_position/local", Odometry, self.OdometryCallback, callback_args=2)
		rospy.Subscriber("yellow/mavros/global_position/local", Odometry, self.OdometryCallback, callback_args=3)
		#rospy.Subscriber("SelectedUAVonGraupner", SelectedUAV, self.SelectedUAVCallback)

		time.sleep(0.5)


	def trajectory_setup(self, req):

		self.selected_UAV = req.selectedUAV
		self.selected_UAV_mode = req.selectedUAV_mode
		traj_wp_x = list(req.desiredX)
		traj_wp_y = list(req.desiredY)
		traj_wp_z = list(req.desiredZ)
		traj_wp_yaw = list(req.desiredW)
		traj_wp_HS = req.desiredHS
		traj_wp_VS = req.desiredVS
		traj_wp_HA = req.desiredHA
		traj_wp_VA = req.desiredVA

		print(traj_wp_yaw)
		traj_wp_yaw = self.fix_angle_discontinuities(traj_wp_yaw)

		# Create a service request which will be filled with waypoints
		request = GenerateTrajectoryRequest()

		# Add waypoints in request
		waypoint = JointTrajectoryPoint()
		#if self.selected_UAV_mode == 1:
		#	num_of_WPs = 2
		#else:
		num_of_WPs = len(traj_wp_x)

		print(num_of_WPs)
		for i in range(0, num_of_WPs):
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

		#x = []; y = []
		#for i in range(len(response.trajectory.points)):
		#	x.append(response.trajectory.points[i].positions[0])
		#	y.append(response.trajectory.points[i].positions[1])
		#plt.plot(x,y, 'b.')
		#plt.show()

		# Response will have trajectory and bool variable success. If for some
		# reason the trajectory was not able to be planned or the configuration
		# was incomplete or wrong it will return False.

		print("Converting trajectory to multi dof")
		joint_trajectory = response.trajectory
		multi_dof_trajectory_data = MultiDOFJointTrajectory()
		multi_dof_trajectory_data = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
		print("Sending trajectory")
		#trajectory_pub.publish(joint_trajectory)

	#Publishing multi_dof_trajectory
	#multi_dof_traj_pub = rospy.Publisher(self.UAV_name+'_sim'+str(self.selected_UAV)+'/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
	#multi_dof_traj_pub.publish(multi_dof_trajectory_msg)

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

			multi_dof_trajectory.points.append(temp_point)

		return multi_dof_trajectory
	

	def fix_angle_discontinuities(self, yaw_list):
		print(1)
		for i in range(1,len(yaw_list)):
			delta = yaw_list[i] - yaw_list[i-1]
			if delta > math.pi:
				for k in range(i,len(yaw_list)):
					yaw_list[k] = yaw_list[k] - 2*math.pi
					print(2)
			elif delta < -math.pi:
				for k in range(i,len(yaw_list)):
					yaw_list[k] = yaw_list[k] + 2*math.pi
					print(3)
		return yaw_list


	def OdometryCallback(self, data, UAV_ID):
		self.position[UAV_ID][0] = data.pose.pose.position.x
		self.position[UAV_ID][1] = data.pose.pose.position.y
		self.position[UAV_ID][2] = data.pose.pose.position.z
		q = data.pose.pose.orientation
		self.position[UAV_ID][3] = atan2( 2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z )


	#def SelectedUAVCallback(self, data):
	#self.selected_UAV = data.selectedUAV
	#self.selected_UAV_mode = data.selectedUAV_mode

	
if __name__ == "__main__":
	rospy.init_node("TrajectoryFromGUIServiceNode")
	RequestTrajectory()
	rospy.spin()
