#include <graupner_serial/Geographiclib_globa2local_service.h>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "geographiclib_global2local_service_node");
    ros::NodeHandle nh;

    // Initialize distance control object
    Geographiclib_global2local_service serv(nh);
	//std::shared_ptr<Geographiclib_global2local_service> global2localObj{ new Geographiclib_global2local_service(nh) };

    double rate = 50;
	ros::Rate loopRate(rate);
	
	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}
}
