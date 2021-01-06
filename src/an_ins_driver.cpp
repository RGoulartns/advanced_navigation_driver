#include "an_ins_driver/an_ins_driver.hpp"


namespace an_ins_driver
{

AnInsDriver::AnInsDriver() :
	Node("an_ins_driver_node"),
	loopFreq_(30)
{

	//ROS PARAMETERS
	comPort_ = this->declare_parameter<std::string>("com_port", "/dev/ttyUSB0");
	baudRate_ = this->declare_parameter<int>("baud_rate", 115200);
	yawOffset_ = this->declare_parameter<double>("yaw_offset", 0.0);

	yawOffset_ = yawOffset_ * M_PI / 180;


    if (OpenComport(const_cast<char*>(comPort_.c_str()), baudRate_))
	{
		RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s \n",comPort_.c_str());
		exit(EXIT_FAILURE);
	}

	an_decoder_initialise(&anDecoder_);

	// Initialise Publishers and Topics //
	imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("ins/imu", 1);
	navSatFix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("ins/fix", 1);
	
	update_tm_ = this->create_wall_timer(
		std::chrono::duration<double>(1.0/loopFreq_),
		std::bind(&AnInsDriver::updateTimerCB, this)
	);

	// Initialise messages
	navSatFix_msg_.header.stamp.sec = 0;
	navSatFix_msg_.header.stamp.nanosec = 0;
	navSatFix_msg_.header.frame_id = "ins";
	navSatFix_msg_.status.status = 0;
	navSatFix_msg_.status.service = 1;
	navSatFix_msg_.latitude = 0.0;
	navSatFix_msg_.longitude = 0.0;
	navSatFix_msg_.altitude = 0.0;
	navSatFix_msg_.position_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	navSatFix_msg_.position_covariance_type = 2; // fixed to variance on the diagonal

	imu_msg_.header.stamp.sec = 0;
	imu_msg_.header.stamp.nanosec = 0;
	imu_msg_.header.frame_id = "ins";
	imu_msg_.orientation.x = 0.0;
	imu_msg_.orientation.y = 0.0;
	imu_msg_.orientation.z = 0.0;
	imu_msg_.orientation.w = 0.0;
	imu_msg_.orientation_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg_.angular_velocity.x = 0.0;
	imu_msg_.angular_velocity.y = 0.0;
	imu_msg_.angular_velocity.z = 0.0;
	imu_msg_.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg_.linear_acceleration.x = 0.0;
	imu_msg_.linear_acceleration.y = 0.0;
	imu_msg_.linear_acceleration.z = 0.0;
	imu_msg_.linear_acceleration_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed

}

AnInsDriver::~AnInsDriver() {}


void AnInsDriver::updateTimerCB()
{
	if ( (bytesReceived_ = PollComport(an_decoder_pointer(&anDecoder_), an_decoder_size(&anDecoder_)) ) > 0)
	{
		// increment the decode buffer length by the number of bytes received //
		an_decoder_increment(&anDecoder_, bytesReceived_);
		
		// decode all the packets in the buffer //
		while ((anPacket_ = an_packet_decode(&anDecoder_)) != NULL)
		{
			// acknowledgement packet //
			if (anPacket_->id == 0) 
				RCLCPP_INFO(this->get_logger(), "acknowledgement data: %d", anPacket_->data[3]);

			// receiver information packet //
			if (anPacket_->id == 69) 
				RCLCPP_INFO(this->get_logger(), "receiver information: %d", anPacket_->data[0]);

			// system state packet //
			if (anPacket_->id == packet_id_system_state) 
			{
				if(decode_system_state_packet(&systemStatePacket_, anPacket_) == 0)
				{
					packetTime_ = this->now();

					// NavSatFix
					navSatFix_msg_.header.stamp = packetTime_;
					if ((systemStatePacket_.filter_status.b.gnss_fix_type == 1) ||  // 2D
						(systemStatePacket_.filter_status.b.gnss_fix_type == 2))   // 3D
					{
						navSatFix_msg_.status.status = 0; // no fix
					}
					else if ((systemStatePacket_.filter_status.b.gnss_fix_type == 3) ||  // SBAS
							(systemStatePacket_.filter_status.b.gnss_fix_type == 5))   // Omnistar/Starfire
					{
						navSatFix_msg_.status.status = 1; // SBAS
					}
					else if ((systemStatePacket_.filter_status.b.gnss_fix_type == 4) ||  // differential
							(systemStatePacket_.filter_status.b.gnss_fix_type == 6) ||  // RTK float
							(systemStatePacket_.filter_status.b.gnss_fix_type == 7))    // RTK fixed
					{
						navSatFix_msg_.status.status = 2; // GBAS
					}
					else 
					{
						navSatFix_msg_.status.status = -1;
					}

					navSatFix_msg_.latitude = systemStatePacket_.latitude * 180.0/M_PI;
					navSatFix_msg_.longitude = systemStatePacket_.longitude * 180.0/M_PI;
					navSatFix_msg_.altitude = systemStatePacket_.height;
					navSatFix_msg_.position_covariance = {
						pow(systemStatePacket_.standard_deviation[1],2), 0.0, 0.0,
						0.0, pow(systemStatePacket_.standard_deviation[0],2), 0.0,
						0.0, 0.0, pow(systemStatePacket_.standard_deviation[2],2)};
					
					// IMU
					imu_msg_.header.stamp = packetTime_;

					// Convert roll, pitch, yaw from radians to quaternion format //
					tf2::Quaternion q;
					q.setRPY(
						systemStatePacket_.orientation[0],
						systemStatePacket_.orientation[1],							
						// 90 degree offset for ENU conversion
						M_PI / 2.0f + yawOffset_ - systemStatePacket_.orientation[2] // REP 103
					);

					imu_msg_.orientation.x = q[0];
					imu_msg_.orientation.y = q[1];
					imu_msg_.orientation.z = q[2];
					imu_msg_.orientation.w = q[3];

					//REP 103: NED to ENU -> x = y, z = -z
					imu_msg_.angular_velocity.x = systemStatePacket_.angular_velocity[1];
					imu_msg_.angular_velocity.y = systemStatePacket_.angular_velocity[0];
					imu_msg_.angular_velocity.z = -systemStatePacket_.angular_velocity[2];
					
					imu_msg_.linear_acceleration.x = systemStatePacket_.body_acceleration[1];
					imu_msg_.linear_acceleration.y = systemStatePacket_.body_acceleration[0];
					imu_msg_.linear_acceleration.z = -systemStatePacket_.body_acceleration[2];
						
				}
			}
			
			// quaternion orientation standard deviation packet //
			if (anPacket_->id == packet_id_euler_orientation_standard_deviation) 
			{
				if(decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet_, anPacket_) == 0)
				{	
					// IMU
					imu_msg.orientation_covariance[0] = euler_orientation_standard_deviation_packet_.standard_deviation[0];
					imu_msg.orientation_covariance[4] = euler_orientation_standard_deviation_packet_.standard_deviation[1];
					imu_msg.orientation_covariance[8] = euler_orientation_standard_deviation_packet_.standard_deviation[2];						
				}
			}

			//angular velocity standard deviation
			if (anPacket_->id == packet_id_velocity_standard_deviation) 
			{
				if(decode_velocity_standard_deviation_packet(&velocity_standard_deviation_packet_, anPacket_) == 0)
				{	
					// IMU
					imu_msg.angular_velocity_covariance[0] = velocity_standard_deviation_packet_.standard_deviation[0];
					imu_msg.angular_velocity_covariance[4] = velocity_standard_deviation_packet_.standard_deviation[1];
					imu_msg.angular_velocity_covariance[8] = velocity_standard_deviation_packet_.standard_deviation[2];						
				}
			}

			// Ensure that you free the anPacket_ when you're done with it (memory leak)
			an_packet_free(&anPacket_);			
		}
	}
	// Publish messages //
	navSatFix_pub_->publish(navSatFix_msg_);
	imu_pub_->publish(imu_msg_);
}

}  // namespace an_ins_driver
