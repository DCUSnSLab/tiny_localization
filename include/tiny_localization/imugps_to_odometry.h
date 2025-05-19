#ifndef IMUGPS_TO_ODOMETRY_H
#define IMUGPS_TO_ODOMETRY_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <GeographicLib/UTMUPS.hpp>
#include <deque>

namespace my_odometry_package {

class IMUGPSToOdometry
{
public:
  IMUGPSToOdometry();
  ~IMUGPSToOdometry();
  void spin();

protected:
  void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void velocityCallback(const std_msgs::Float64::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  double normalizeAngle(double angle);
  double angleDiff(double a, double b);
  double northDegreeToEastRadians(double degrees);
  geometry_msgs::Quaternion northQuaternionToEastQuaternion(const geometry_msgs::Quaternion& q);
  void convertLatLonToUTM(double lat, double lon, double &easting, double &northing);
  bool calculateGPSHeading(double &heading_rad);

  ros::NodeHandle nh_;
  ros::Subscriber gps_fix_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber velocity_sub_;

  ros::Publisher current_utm_position_pub_;
  ros::Publisher odom_pub_ekf_;

  sensor_msgs::NavSatFix gps_fix_msg_;
  sensor_msgs::Imu imu_msg_;

  std::deque<std::pair<double, double>> gps_utm_history_;
  ros::Time last_gps_time_;
  double min_gps_distance_for_heading_;
  double min_speed_for_heading_;
  
  geometry_msgs::Point init_position_utm_;
  bool gps_fix_received_;
  bool init_position_flag_;

  double last_time_;

  // EKF variables
  bool ekf_initialized_;
  Eigen::Vector3d ekf_state_;  // [x, y, heading]
  Eigen::Matrix3d ekf_P_;      // Covariance matrix
  Eigen::Matrix3d Q_;          // Process noise covariance
  Eigen::Matrix2d R_pos_;      // Position measurement noise covariance
  double R_heading_;           // Heading measurement noise variance

  double gps_heading_meas_;
  bool new_gps_position_;
  bool new_gps_heading_;

  double current_speed_;
};

}

#endif
