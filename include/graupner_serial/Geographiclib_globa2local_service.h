#include <ros/ros.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <graupner_serial/LatLong.h>

class Geographiclib_global2local_service
{
    private:
        ros::ServiceServer _service;
        GeographicLib::Geocentric *_earth;
        GeographicLib::LocalCartesian *_earth_local;
    public:
        //Geographiclib_global2local_service();
        Geographiclib_global2local_service(ros::NodeHandle &nh);


    bool callback(graupner_serial::LatLong::Request& request, 
                  graupner_serial::LatLong::Response& response);

};
