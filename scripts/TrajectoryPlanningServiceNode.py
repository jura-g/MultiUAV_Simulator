#!/usr/bin/env python

import copy, time

# Ros imports
import rospy, math

from math import sin, cos

from nav_msgs.msg import Odometry

from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
from mavros_msgs.msg import HomePosition

from graupner_serial.msg import SelectedUAV

from graupner_serial.srv import BuildingDescriptor, BuildingTrajectoryParameters, LatLong
from math import atan2, pi
import matplotlib.pyplot as plt
from math import sin, cos


class RequestTrajectory():

    def __init__(self):

    	self.number_of_UAVs = 3
	self.selected_UAV = 0
	self.position = [[0.0 for x in range(self.number_of_UAVs+1)] for y in range(4)]
	for i in range(self.number_of_UAVs+1):
		self.position[i][2] = 1.0

        # First set up service
	self.service = rospy.Service('TrajectoryPlanningService', BuildingTrajectoryParameters, self.trajectory_setup)

        self.request_geolib_service = rospy.ServiceProxy("geographiclib_global2local_service", LatLong)

	self.request_generate_trajectoy_service = rospy.ServiceProxy("generate_trajectory_points_service", BuildingDescriptor)

        self.request_trajectory_service = rospy.ServiceProxy("generate_toppra_trajectory", GenerateTrajectory)

        # This is an example for UAV and output trajectory is converted 
        # accordingly
        #trajectory_pub = rospy.Publisher('/building_envelope/joint_trajectory', 
        #    JointTrajectory, queue_size=100)

        #home_pub = rospy.Publisher('/mavros/global_position/home', 
        #    HomePosition, queue_size=1)
        
        #self.position = [0.0, 0.0, 1.0, 0.0]

       
	#uav_pose_sub = rospy.Subscriber('/red/carrot/pose',
        #    PoseStamped, pose_update)

	self.UAV_name = rospy.get_param('uav_name_param')
	rospy.Subscriber(self.UAV_name+"_sim1/odometry_sensor1/odometry", Odometry, self.OdometryCallback, callback_args=1)
	rospy.Subscriber(self.UAV_name+"_sim2/odometry_sensor1/odometry", Odometry, self.OdometryCallback, callback_args=2)
	rospy.Subscriber(self.UAV_name+"_sim3/odometry_sensor1/odometry", Odometry, self.OdometryCallback, callback_args=3)
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
	res_building_points = 0.3
	res_height = 10
	wp_file_path = req.wp_file_path
	coord_sys_text = req.coord_sys_text

        #latitude = [45.807742, 45.807643, 45.808806, 45.808866]    # mimara (u 4 tocke)
        #longitude = [15.966094, 15.967185, 15.967314, 15.966224]
        #latitude = [45.807742, 45.807733, 45.807794, 45.807800]    # mali cetverokut (7x7 m)
        #longitude = [15.966094, 15.966196, 15.966208, 15.966114]

	if coord_sys_text == 'WGS84':
		latitude = x
        	longitude = y
		resp = self.request_geolib_service(latitude, longitude, latitude[0], longitude[0])
		x = resp.x
		y = resp.y

        resp = self.request_generate_trajectoy_service(x, y, height, dist, res, res_building_points, res_height)
	traj_wp_x = [self.position[self.selected_UAV][0]] + list(resp.traj_x)
        traj_wp_y = [self.position[self.selected_UAV][1]] + list(resp.traj_y)
        traj_wp_z = [self.position[self.selected_UAV][2]] + list(resp.traj_z)
        traj_wp_yaw = [self.position[self.selected_UAV][3]] + list(resp.traj_yaw)

        traj_wp_yaw = self.fix_angle_discontinuities(traj_wp_yaw, res, dist)

	with open(wp_file_path, 'w') as f:
		for i in range(len(traj_wp_x)):
			f.write("%d,%f,%f,%f,%f\n" % (i+1, traj_wp_x[i], traj_wp_y[i], traj_wp_z[i], traj_wp_yaw[i]))

        #plotx = []; ploty = []
        #for i in range(len(traj_wp_yaw)):
        #    plotx.append(traj_wp_x[i] + cos(traj_wp_yaw[i]))
        #    ploty.append(traj_wp_y[i] + sin(traj_wp_yaw[i]))
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
                waypoint.velocities = [1, 1, 1, 1]
                waypoint.accelerations = [0.5, 0.5, 0.5, 0.3]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint))

        # Set up joint names. This step is not necessary
        request.waypoints.joint_names = ["x", "y", "z", "yaw"]
        # Set up sampling frequency of output trajectory.
        request.sampling_frequency = 100.0
        # Set up number of gridpoints. The more gridpoints there are, 
        # trajectory interpolation will be more accurate but slower.
        # Defaults to 100
        request.n_gridpoints = 2000
        # If you want to plot Maximum Velocity Curve and accelerations you can
        # send True in this field. This is intended to be used only when you
        # have to debug something since it will block the service until plot
        # is closed.
        request.plot = False
        # Request the trajectory
        response = self.request_trajectory_service(request)

        #x = []; y = []
        #for i in range(len(response.trajectory.points)):
        #    x.append(response.trajectory.points[i].positions[0])
        #    y.append(response.trajectory.points[i].positions[1])
        #plt.plot(x,y, 'b.')
        #plt.show()

        # Response will have trajectory and bool variable success. If for some
        # reason the trajectory was not able to be planned or the configuration
        # was incomplete or wrong it will return False.

        print("Converting trajectory to multi dof")
        joint_trajectory = response.trajectory
	multi_dof_trajectory_msg = MultiDOFJointTrajectory()
        multi_dof_trajectory_msg = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
        print("Sending trajectory")
        #trajectory_pub.publish(joint_trajectory)

	#Publishing multi_dof_trajectory
	multi_dof_traj_pub = rospy.Publisher(self.UAV_name+'_sim'+str(self.selected_UAV)+'/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
	multi_dof_traj_pub.publish(multi_dof_trajectory_msg)

	return traj_wp_x, traj_wp_y, traj_wp_z, traj_wp_yaw        

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
    

    def fix_angle_discontinuities(self, alphas, res, dist):
        cnt = 0
        for i in range(len(alphas)-1):
            if alphas[i+1]+cnt*2*pi - alphas[i] > 2*res/dist:
                cnt -= 1
            if alphas[i+1]+cnt*2*pi - alphas[i] < -2*res/dist:
                cnt += 1

            alphas[i+1] += cnt * 2*pi
        return alphas


    def OdometryCallback(self, data, UAV_ID):
        self.position[UAV_ID][0] = data.pose.pose.position.x
        self.position[UAV_ID][1] = data.pose.pose.position.y
        self.position[UAV_ID][2] = data.pose.pose.position.z
        q = data.pose.pose.orientation
        self.position[UAV_ID][3] = atan2( 2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z )


    def SelectedUAVCallback(self, data):
	self.selected_UAV = data.selectedUAV
	#self.selected_UAV_mode = data.selectedUAV_mode

	
if __name__ == "__main__":
    rospy.init_node("TrajectoryPlanningServiceNode")
    RequestTrajectory()
    rospy.spin()
