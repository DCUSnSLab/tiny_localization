#include "imugps_to_odometry.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_converter_ekf");
  my_odometry_package::IMUGPSToOdometry node;
  node.spin();
  return 0;
}