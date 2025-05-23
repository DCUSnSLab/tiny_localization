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

namespace tiny_localization {

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
  
  // Add data check method
  void checkDataTimeouts();

  double normalizeAngle(double angle);
  double angleDiff(double a, double b);
  double northDegreeToEastRadians(double degrees);
  geometry_msgs::Quaternion northQuaternionToEastQuaternion(const geometry_msgs::Quaternion& q);
  void convertLatLonToUTM(double lat, double lon, double &easting, double &northing);
  bool calculateGPSHeading(double &heading_rad);

  int gps_not_recv_count_{0};
  int imu_not_recv_count_{0};
  int vel_not_recv_count_{0};

  bool all_data_callback_received_{false};
  bool all_data_callback_received_flag_{false};
  int timeout_count_th_{1};

  ros::NodeHandle nh_;
  ros::Subscriber gps_fix_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber velocity_sub_;
  
  // Add timer for checking data timeouts
  ros::Timer timeout_check_timer_;
  
  // Add last message received times
  ros::Time last_gps_received_time_;
  ros::Time last_imu_received_time_;
  ros::Time last_velocity_received_time_;
  
  // Add timeout thresholds (in seconds)
  double gps_timeout_threshold_;
  double imu_timeout_threshold_;
  double velocity_timeout_threshold_;

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
