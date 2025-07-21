#include "tiny_localization/imugps_to_odometry.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tiny_localization::IMUGPSToOdometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}