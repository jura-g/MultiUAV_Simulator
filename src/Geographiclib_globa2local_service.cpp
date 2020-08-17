#include <graupner_serial/Geographiclib_globa2local_service.h>
#include <Eigen/Dense>
#include <GeographicLib/Geocentric.hpp>
#include <mavros/frame_tf.h>


Geographiclib_global2local_service::Geographiclib_global2local_service(ros::NodeHandle &nh)
{
    _earth = new GeographicLib::Geocentric(GeographicLib::Constants::WGS84_a(), 
									 	   GeographicLib::Constants::WGS84_f());

	_earth_local = new GeographicLib::LocalCartesian(*_earth);
	
	// Initialize service
	_service = nh.advertiseService( "geographiclib_global2local_service",
			                        &Geographiclib_global2local_service::callback,
									this);					 
}

Eigen::Vector3d toLocal(double latitude0, double longitude0, double altitude0, const double lat, const double lon, const double alt, bool altitudeRelative = false) {
  Eigen::Vector3d local_ecef;
  try {
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f());

    Eigen::Vector3d ecef_origin, 
    map_origin(
      //m_homeHandler.getData().geo.latitude, 
      //m_homeHandler.getData().geo.longitude, 
      //m_homeHandler.getData().geo.altitude);
	  latitude0, 
      longitude0, 
      altitude0);
    
    earth.Forward(map_origin.x(), map_origin.y(), map_origin.z(), ecef_origin.x(), ecef_origin.y(), ecef_origin.z());
    
    earth.Forward(lat, lon, map_origin.z(), local_ecef.x(), local_ecef.y(), local_ecef.z());   
    
    local_ecef = local_ecef - ecef_origin;
    local_ecef = mavros::ftf::transform_frame_ecef_enu(local_ecef, map_origin);
  } catch (const std::exception& e) {
    ROS_FATAL("Global2Local::toLocal - unable to perform transformation");
  }

  if (altitudeRelative) {
    local_ecef.z() = alt;
  }

  //if (!m_homeHandler.isMessageRecieved()) {
  //  ROS_FATAL("Global2Local - unable to get home position.");
  //  local_ecef.x() = 0;
  //  local_ecef.y() = 0;
  //}

  return local_ecef;
}

bool Geographiclib_global2local_service::callback(graupner_serial::LatLong::Request& request, 
												  graupner_serial::LatLong::Response& response)
{
	double height = 0;
	//_earth_local->Reset(request.lattitude0, request.longitude0, height);

	double x, y, z;
	Eigen::Vector3d local_point;
	for(int i = 0; i<request.longitude.size(); i++){
		local_point = toLocal(request.latitude0, request.longitude0, request.altitude0, request.latitude[i], request.longitude[i], height, true);
		//_earth_local->Forward(request.lattitude[i], request.longitude[i], 0, x, y, z);
		response.x.push_back(local_point.x());
		response.y.push_back(local_point.y());
	}
	return true;
}											  
