#include <graupner_serial/Geographiclib_globa2local_service.h>


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

bool Geographiclib_global2local_service::callback(graupner_serial::LatLong::Request& request, 
												  graupner_serial::LatLong::Response& response)
{
	double height = 0;
	_earth_local->Reset(request.lattitude0, request.longitude0, height);

	double x, y, z;
	for(int i = 0; i<request.longitude.size(); i++){
		_earth_local->Forward(request.lattitude[i], request.longitude[i], 0,
								x, y, z);

		response.x.push_back(x);
		response.y.push_back(y);
	}
	return true;
}											  
