/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2017, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/String.h>

#include <cmath>	  
#include <cstdio>

#define fr 298.257223563
#define a 6378137
#define k0 0.9996
#define RADIANS_TO_DEGREES (180.0/M_PI)
const double PI       =	4*atan(1);	//Gantz used: PI=3.14159265;
const double deg2rad  = PI/180;
const double ee = 2/fr-1/(fr*fr);
const double EE = ee/(1-ee);
double LongOriginRad;
double yaw_offset = 0;

int main(int argc, char *argv[])
{
	// Set up ROS node //
	ros::init(argc, argv, "an_device");
	ros::NodeHandle nh("~");
	ros::Rate loopRate(30);

	// Set up the COM port
	std::string com_port_s;
	nh.param<std::string>("port", com_port_s, "/dev/ttyS0");
	char *com_port = (char *)com_port_s.c_str();

	int baud_rate;
	nh.param<int>("baud", baud_rate, 115200);

	if (OpenComport(com_port, baud_rate))
	{
		ROS_INFO("Could not open serial port %s at %d baud.", com_port, baud_rate);
		exit(EXIT_FAILURE);
	}
	ROS_INFO("port:%s@%d", com_port, baud_rate);

	
	// If a UTM Zone is provided, publish transforms.
	// The zone is static to avoid problems due to changing when near a Zone boundary.
	int utm_zone;
	std::string tf_name = nh.getNamespace();
	tf::Transform transform;
	if (nh.getParam("utm_zone", utm_zone)) {
		LongOriginRad = (utm_zone*6 - 183) * deg2rad;
		ROS_INFO("using UTM Zone %d to publish transform %s", utm_zone, tf_name.c_str());

	}

	if (nh.getParam("yaw_offset", yaw_offset)) {
		yaw_offset = yaw_offset * deg2rad;
		ROS_INFO("using %f as the yaw offset", yaw_offset);

	}


	// Initialise Publishers, and Subscribers //
	ros::Publisher nav_sat_fix_pub=nh.advertise<sensor_msgs::NavSatFix>("ins/fix",10);
	ros::Publisher imu_pub=nh.advertise<sensor_msgs::Imu>("/ins/imu",10);

	// Initialise messages
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec=0;
	nav_sat_fix_msg.header.stamp.nsec=0;
	nav_sat_fix_msg.header.frame_id="gps"; // fixed
	nav_sat_fix_msg.status.status=0;
	nav_sat_fix_msg.status.service=1; // fixed to GPS
	nav_sat_fix_msg.latitude=0.0;
	nav_sat_fix_msg.longitude=0.0;
	nav_sat_fix_msg.altitude=0.0;
	nav_sat_fix_msg.position_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type=2; // fixed to variance on the diagonal

	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x=0.0;
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp.sec=0;
	imu_msg.header.stamp.nsec=0;
	imu_msg.header.frame_id="imu"; // fixed
	imu_msg.orientation.x=0.0;
	imu_msg.orientation.y=0.0;
	imu_msg.orientation.z=0.0;
	imu_msg.orientation.w=0.0;
	imu_msg.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x=0.0;
	imu_msg.angular_velocity.y=0.0;
	imu_msg.angular_velocity.z=0.0;
	imu_msg.angular_velocity_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg.linear_acceleration.x=0.0;
	imu_msg.linear_acceleration.y=0.0;
	imu_msg.linear_acceleration.z=0.0;
	imu_msg.linear_acceleration_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	
	diagnostic_msgs::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";
	
	diagnostic_msgs::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";
	
	// get data from com port //
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet;
	velocity_standard_deviation_packet_t velocity_standard_deviation_packet;

	int bytes_received;
	

	an_decoder_initialise(&an_decoder);

	bool offsetMeasured = false;	
	ros::Time timeNow;
	
	// Loop continuously, polling for packets
	while (ros::ok())
	{
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			// increment the decode buffer length by the number of bytes received //
			an_decoder_increment(&an_decoder, bytes_received);


			
			// decode all the packets in the buffer //
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				// acknowledgement packet //
				if (an_packet->id == 0) 
				{
					ROS_INFO("acknowledgement data: %d", an_packet->data[3]);
				}

				// receiver information packet //
				if (an_packet->id == 69) 
				{
					ROS_INFO("receiver information: %d", an_packet->data[0]);
				}

				// system state packet //
				if (an_packet->id == packet_id_system_state) 
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						timeNow = ros::Time::now();

						// NavSatFix
						nav_sat_fix_msg.header.stamp.sec = timeNow.sec;
						nav_sat_fix_msg.header.stamp.nsec = timeNow.nsec;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||  // 2D
							(system_state_packet.filter_status.b.gnss_fix_type == 2))   // 3D
						{
							nav_sat_fix_msg.status.status=0; // no fix
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||  // SBAS
							 (system_state_packet.filter_status.b.gnss_fix_type == 5))   // Omnistar/Starfire
						{
							nav_sat_fix_msg.status.status=1; // SBAS
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||  // differential
							 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||  // RTK float
							 (system_state_packet.filter_status.b.gnss_fix_type == 7))    // RTK fixed
						{
							nav_sat_fix_msg.status.status=2; // GBAS
						}
						else 
						{
							nav_sat_fix_msg.status.status=-1;
						}
						nav_sat_fix_msg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude=system_state_packet.height;
						nav_sat_fix_msg.position_covariance={pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
							0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
							0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};
							
						// IMU
						imu_msg.header.stamp.sec = timeNow.sec;
						imu_msg.header.stamp.nsec = timeNow.nsec;

						// Convert roll, pitch, yaw from radians to quaternion format //
						tf::Quaternion orientation;
						orientation.setRPY(
							system_state_packet.orientation[0],
							system_state_packet.orientation[1],
							// 90 degree offset for ENU conversion
						    PI / 2.0f + yaw_offset - system_state_packet.orientation[2] // REP 103
						);

						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];

						imu_msg.angular_velocity.x = system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.y = system_state_packet.angular_velocity[0];
						imu_msg.angular_velocity.z = -system_state_packet.angular_velocity[2];
						
						imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[1];
						imu_msg.linear_acceleration.y = system_state_packet.body_acceleration[0];
						imu_msg.linear_acceleration.z = -system_state_packet.body_acceleration[2];
						
					}
				}
				
				// quaternion orientation standard deviation packet //
				if (an_packet->id == packet_id_euler_orientation_standard_deviation) 
				{
					if(decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
					{	
						// IMU
						imu_msg.orientation_covariance[0] = euler_orientation_standard_deviation_packet.standard_deviation[0];
						imu_msg.orientation_covariance[4] = euler_orientation_standard_deviation_packet.standard_deviation[1];
						imu_msg.orientation_covariance[8] = euler_orientation_standard_deviation_packet.standard_deviation[2];						
					}
				}

				//angular velocity standard deviation
				if (an_packet->id == packet_id_velocity_standard_deviation) 
				{
					if(decode_velocity_standard_deviation_packet(&velocity_standard_deviation_packet, an_packet) == 0)
					{	
						// IMU
						imu_msg.angular_velocity_covariance[0] = velocity_standard_deviation_packet.standard_deviation[0];
						imu_msg.angular_velocity_covariance[4] = velocity_standard_deviation_packet.standard_deviation[1];
						imu_msg.angular_velocity_covariance[8] = velocity_standard_deviation_packet.standard_deviation[2];						
					}
				}

				// Ensure that you free the an_packet when you're done with it //
				// or you will leak memory                                   //
				an_packet_free(&an_packet);
			}
		}
		nav_sat_fix_pub.publish(nav_sat_fix_msg);
		imu_pub.publish(imu_msg);
		ros::spinOnce();
		loopRate.sleep();
	}

}
