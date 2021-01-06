#include "an_ins_driver/an_ins_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<an_ins_driver::AnInsDriver>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  
  return 0;
}
