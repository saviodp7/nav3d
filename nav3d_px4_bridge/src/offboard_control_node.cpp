#include "nav3d_px4_bridge/offboard_control.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav3d_px4_bridge::OffboardControl>());
  rclcpp::shutdown();
  return 0;
}