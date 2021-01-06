// Use this node to calculate an initial gps fix using a median value and send it to robot_localization.


#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include "geographic_msgs/GeoPose.h"
#include "robot_localization/navsat_transform.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/filter_utilities.h"
#include "robot_localization/navsat_conversions.h"
#include "robot_localization/ros_filter_utilities.h"
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>


std::vector< double > latVect, longVect, altVect;
geographic_msgs::GeoPose geoMsg;


void gpsCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
	latVect.push_back(msg->latitude);
	longVect.push_back(msg->longitude);
	altVect.push_back(msg->altitude);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "imu_read");
	ros::NodeHandle n;
	ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("/ins/fix", 10, &gpsCallback);
	ros::ServiceClient client = n.serviceClient<robot_localization::SetDatum>("datum");

	ros::Rate loopRate(30);

	bool initialMedian = false;
	int size;
	double latMedian = 0, longMedian = 0, altMedian = 0;
	
	while( ros::ok() )
	{
		size = latVect.size();
		if ( (size > 300) && !initialMedian)
		{						
			std::sort(latVect.begin(), latVect.end());
			std::sort(longVect.begin(), longVect.end());
			std::sort(altVect.begin(), altVect.end());
			latMedian = latVect[ size/2 ];
			longMedian = longVect[ size/2 ];
			altMedian = altVect[ size/2 ];
			initialMedian = true;

			geoMsg.position.latitude = latMedian;
			geoMsg.position.longitude = longMedian;
			geoMsg.position.altitude = altMedian;
			geoMsg.orientation.x = 0;
			geoMsg.orientation.y = 0;
			geoMsg.orientation.z = 0;
			geoMsg.orientation.w = 1;
			robot_localization::SetDatum srv;
			srv.request.geo_pose = geoMsg;
			client.call(srv);
			if (client.call(srv))
				ROS_INFO("Service worked");
			break;
		}
		
		ros::spinOnce();
		loopRate.sleep();
	}	

}
