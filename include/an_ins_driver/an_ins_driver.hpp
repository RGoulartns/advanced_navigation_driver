#ifndef AN_INS_DRIVER__AN_INS_DRIVER_HPP_
#define AN_INS_DRIVER__AN_INS_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

namespace an_ins_driver
{

class AnInsDriver : public rclcpp::Node
{
public:
  AnInsDriver();

  virtual ~AnInsDriver();

private:
  void updateTimerCB();

  // Advanced Navigation Variables
	an_decoder_t anDecoder_;
	an_packet_t *anPacket_;
  system_state_packet_t systemStatePacket_;
	euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet_;
  velocity_standard_deviation_packet_t velocity_standard_deviation_packet_;
	int bytesReceived_;
  rclcpp::Time packetTime_;

  // ROS Parameters
  std::string comPort_;
  int baudRate_;
  short unsigned int loopFreq_;
  double yawOffset_;

  // ROS messages
  sensor_msgs::msg::NavSatFix navSatFix_msg_;
  sensor_msgs::msg::Imu imu_msg_;

  // ROS publishers and Subscribers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFix_pub_;

  rclcpp::TimerBase::SharedPtr update_tm_;

};

}  // namespace an_ins

#endif  // AN_INS_DRIVER__AN_INS_DRIVER_HPP_
