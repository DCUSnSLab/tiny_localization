#ifndef IMUGPS_TO_ODOMETRY_H
#define IMUGPS_TO_ODOMETRY_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eigen3/Eigen/Dense>
#ifdef HAS_GEOGRAPHICLIB
#include <GeographicLib/UTMUPS.hpp>
#endif
#include <deque>

namespace tiny_localization {

class IMUGPSToOdometry : public rclcpp::Node
{
public:
  IMUGPSToOdometry();
  ~IMUGPSToOdometry() = default;

protected:
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  // Add data check method
  void checkDataTimeouts();

  double normalizeAngle(double angle);
  double angleDiff(double a, double b);
  double northDegreeToEastRadians(double degrees);
  geometry_msgs::msg::Quaternion northQuaternionToEastQuaternion(const geometry_msgs::msg::Quaternion& q);
  void convertLatLonToUTM(double lat, double lon, double &easting, double &northing);
  bool calculateGPSHeading(double &heading_rad);

  int gps_not_recv_count_{0};
  int imu_not_recv_count_{0};
  int vel_not_recv_count_{0};

  bool all_data_callback_received_{false};
  bool all_data_callback_received_flag_{false};
  int timeout_count_th_{1};

  // ROS2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
  
  // Add timer for checking data timeouts
  rclcpp::TimerBase::SharedPtr timeout_check_timer_;
  
  // Add last message received times
  rclcpp::Time last_gps_received_time_;
  rclcpp::Time last_imu_received_time_;
  rclcpp::Time last_velocity_received_time_;
  
  // Add timeout thresholds (in seconds)
  double gps_timeout_threshold_;
  double imu_timeout_threshold_;
  double velocity_timeout_threshold_;

  // ROS2 publishers
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_utm_position_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ekf_;

  sensor_msgs::msg::NavSatFix gps_fix_msg_;
  sensor_msgs::msg::Imu imu_msg_;

  std::deque<std::pair<double, double>> gps_utm_history_;
  rclcpp::Time last_gps_time_;
  double min_gps_distance_for_heading_;
  double min_speed_for_heading_;
  
  geometry_msgs::msg::Point init_position_utm_;
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