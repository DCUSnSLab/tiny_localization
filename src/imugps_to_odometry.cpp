#include "tiny_localization/imugps_to_odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sstream>
#ifdef HAS_GEOGRAPHICLIB
#include <GeographicLib/UTMUPS.hpp>
#endif
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace tiny_localization {

IMUGPSToOdometry::IMUGPSToOdometry() : Node("imugps_to_odometry"),
  gps_fix_received_(false),
  init_position_flag_(true),
  last_time_(0.0),
  ekf_initialized_(false),
  gps_heading_meas_(0.0),
  new_gps_position_(false),
  new_gps_heading_(false),
  current_speed_(0.0),
  min_gps_distance_for_heading_(0.0),
  gps_timeout_threshold_(1.0),
  imu_timeout_threshold_(0.5),
  velocity_timeout_threshold_(1.0)
{
  // Topic settings
  std::string gps_fix_topic = "/ublox_gps/fix";
  std::string imu_topic = "/vectornav/IMU";
  std::string velocity_topic = "/vehicle/velocity";
  std::string output_odom_topic = "/odom/ekf";
  std::string output_utm_topic = "/current_utm_relative_position";

  // Declare parameters
  this->declare_parameter("topics.gps_fix_topic", gps_fix_topic);
  this->declare_parameter("topics.imu_topic", imu_topic);
  this->declare_parameter("topics.velocity_topic", velocity_topic);
  this->declare_parameter("topics.output_odom_topic", output_odom_topic);
  this->declare_parameter("topics.output_utm_topic", output_utm_topic);
  
  // Declare initial position parameters
  this->declare_parameter("init_position.x", 0.0);
  this->declare_parameter("init_position.y", 0.0);
  this->declare_parameter("init_position.z", 0.0);

  // GPS heading settings
  this->declare_parameter("gps_heading.min_distance_for_heading", 0.0);
  this->declare_parameter("gps_heading.min_speed_for_heading", 0.6);

  // EKF process noise settings
  this->declare_parameter("process_noise.x", 0.05);
  this->declare_parameter("process_noise.y", 0.05);
  this->declare_parameter("process_noise.yaw", 0.001);

  // EKF measurement noise settings
  this->declare_parameter("measurement_noise.position.x", 0.1);
  this->declare_parameter("measurement_noise.position.y", 0.1);
  this->declare_parameter("measurement_noise.heading", 0.7);

  // Initial EKF settings
  this->declare_parameter("ekf_initial.position_uncertainty", 1.0);
  this->declare_parameter("ekf_initial.heading_uncertainty", 1.0);

  // Get parameters
  gps_fix_topic = this->get_parameter("topics.gps_fix_topic").as_string();
  imu_topic = this->get_parameter("topics.imu_topic").as_string();
  velocity_topic = this->get_parameter("topics.velocity_topic").as_string();
  output_odom_topic = this->get_parameter("topics.output_odom_topic").as_string();
  output_utm_topic = this->get_parameter("topics.output_utm_topic").as_string();

  min_gps_distance_for_heading_ = this->get_parameter("gps_heading.min_distance_for_heading").as_double();
  min_speed_for_heading_ = this->get_parameter("gps_heading.min_speed_for_heading").as_double();

  double process_noise_x = this->get_parameter("process_noise.x").as_double();
  double process_noise_y = this->get_parameter("process_noise.y").as_double();
  double process_noise_yaw = this->get_parameter("process_noise.yaw").as_double();

  double measurement_noise_pos_x = this->get_parameter("measurement_noise.position.x").as_double();
  double measurement_noise_pos_y = this->get_parameter("measurement_noise.position.y").as_double();
  R_heading_ = this->get_parameter("measurement_noise.heading").as_double();

  double position_uncertainty = this->get_parameter("ekf_initial.position_uncertainty").as_double();
  double heading_uncertainty = this->get_parameter("ekf_initial.heading_uncertainty").as_double();

  RCLCPP_INFO(this->get_logger(), "GPS fix topic: %s", gps_fix_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Velocity topic: %s", velocity_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Output odom topic: %s", output_odom_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Output UTM topic: %s", output_utm_topic.c_str());

  // Initialize EKF matrices
  Q_ = Eigen::Matrix3d::Zero();
  Q_(0, 0) = process_noise_x;
  Q_(1, 1) = process_noise_y;
  Q_(2, 2) = process_noise_yaw;

  R_pos_ = Eigen::Matrix2d::Zero();
  R_pos_(0, 0) = measurement_noise_pos_x;
  R_pos_(1, 1) = measurement_noise_pos_y;

  ekf_P_ = Eigen::Matrix3d::Zero();
  ekf_P_(0, 0) = position_uncertainty;
  ekf_P_(1, 1) = position_uncertainty;
  ekf_P_(2, 2) = heading_uncertainty;

  ekf_state_ = Eigen::Vector3d::Zero();

  // Initialize subscribers
  gps_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_fix_topic, 10, std::bind(&IMUGPSToOdometry::gpsFixCallback, this, std::placeholders::_1));
  
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 10, std::bind(&IMUGPSToOdometry::imuCallback, this, std::placeholders::_1));
  
  velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    velocity_topic, 10, std::bind(&IMUGPSToOdometry::velocityCallback, this, std::placeholders::_1));

  // Initialize publishers
  current_utm_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(output_utm_topic, 10);
  odom_pub_ekf_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic, 10);

  // Initialize timer for timeout checking
  timeout_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&IMUGPSToOdometry::checkDataTimeouts, this));

  // Initialize timestamps
  last_gps_received_time_ = this->now();
  last_imu_received_time_ = this->now();
  last_velocity_received_time_ = this->now();
  last_gps_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "IMU-GPS to Odometry node initialized successfully");
}

void IMUGPSToOdometry::gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  last_gps_received_time_ = this->now();
  gps_not_recv_count_ = 0;
  
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GPS No Fix");
    return;
  }

  gps_fix_msg_ = *msg;
  gps_fix_received_ = true;

  double easting, northing;
  convertLatLonToUTM(msg->latitude, msg->longitude, easting, northing);

  if (init_position_flag_) {
    RCLCPP_INFO(this->get_logger(), "Initializing position with first GPS fix...");
    rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for 1 second

    // Set initial position parameter using actual GPS coordinates
    this->set_parameter(rclcpp::Parameter("init_position.x", easting));
    this->set_parameter(rclcpp::Parameter("init_position.y", northing));
    this->set_parameter(rclcpp::Parameter("init_position.z", 0.0));

    init_position_utm_.x = easting;
    init_position_utm_.y = northing;
    init_position_utm_.z = 0.0;
    init_position_flag_ = false;
    
    ekf_state_(0) = 0.0;
    ekf_state_(1) = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Initial UTM position set: (%.2f, %.2f)", easting, northing);
    
    gps_utm_history_.clear();
    gps_utm_history_.push_back(std::make_pair(easting, northing));
    last_gps_time_ = this->now();
    return;
  }

  // Calculate relative position
  double rel_x = easting - init_position_utm_.x;
  double rel_y = northing - init_position_utm_.y;

  // Add to GPS history for heading calculation
  gps_utm_history_.push_back({rel_x, rel_y});
  if (gps_utm_history_.size() > 10) {
    gps_utm_history_.pop_front();
  }

  // Calculate GPS heading
  double gps_heading_rad;
  if (calculateGPSHeading(gps_heading_rad)) {
    gps_heading_meas_ = gps_heading_rad;
    new_gps_heading_ = true;
  }

  new_gps_position_ = true;
  last_gps_time_ = this->now();

  // Publish current UTM position
  geometry_msgs::msg::Point utm_pos;
  utm_pos.x = rel_x;
  utm_pos.y = rel_y;
  utm_pos.z = 0.0;
  current_utm_position_pub_->publish(utm_pos);
}

void IMUGPSToOdometry::velocityCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_velocity_received_time_ = this->now();
  vel_not_recv_count_ = 0;
  current_speed_ = msg->data;
}

void IMUGPSToOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  last_imu_received_time_ = this->now();
  imu_not_recv_count_ = 0;
  
  if (!gps_fix_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for GPS fix");
    return;
  }

  imu_msg_ = *msg;
  
  double current_time = this->now().seconds();
  
  if (last_time_ == 0.0) {
    last_time_ = current_time;
    return;
  }

  double dt = current_time - last_time_;
  last_time_ = current_time;

  if (dt <= 0.0 || dt > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid dt: %f", dt);
    return;
  }

  // Initialize EKF if not already done
  if (!ekf_initialized_) {
    if (new_gps_heading_) {
      ekf_state_(2) = gps_heading_meas_;
      ekf_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "EKF initialized with heading: %.3f rad", gps_heading_meas_);
    } else {
      return;
    }
  }

  // EKF Prediction Step
  double omega_z = msg->angular_velocity.z;
  
  // State prediction
  Eigen::Vector3d state_pred = ekf_state_;
  state_pred(0) += current_speed_ * cos(ekf_state_(2)) * dt;
  state_pred(1) += current_speed_ * sin(ekf_state_(2)) * dt;
  state_pred(2) += omega_z * dt;
  state_pred(2) = normalizeAngle(state_pred(2));

  // Jacobian of state transition
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = -current_speed_ * sin(ekf_state_(2)) * dt;
  F(1, 2) = current_speed_ * cos(ekf_state_(2)) * dt;

  // Covariance prediction
  Eigen::Matrix3d P_pred = F * ekf_P_ * F.transpose() + Q_;

  // EKF Update Step
  Eigen::Vector3d state_update = state_pred;
  Eigen::Matrix3d P_update = P_pred;

  // GPS position update
  if (new_gps_position_) {
    double rel_x = (gps_fix_msg_.latitude - init_position_utm_.x);
    double rel_y = (gps_fix_msg_.longitude - init_position_utm_.y);
    
    double easting, northing;
    convertLatLonToUTM(gps_fix_msg_.latitude, gps_fix_msg_.longitude, easting, northing);
    rel_x = easting - init_position_utm_.x;
    rel_y = northing - init_position_utm_.y;

    Eigen::Vector2d z_pos(rel_x, rel_y);
    Eigen::Vector2d h_pos(state_pred(0), state_pred(1));
    Eigen::Vector2d residual_pos = z_pos - h_pos;

    Eigen::Matrix<double, 2, 3> H_pos = Eigen::Matrix<double, 2, 3>::Zero();
    H_pos(0, 0) = 1.0;
    H_pos(1, 1) = 1.0;

    Eigen::Matrix2d S_pos = H_pos * P_pred * H_pos.transpose() + R_pos_;
    Eigen::Matrix<double, 3, 2> K_pos = P_pred * H_pos.transpose() * S_pos.inverse();

    state_update = state_pred + K_pos * residual_pos;
    P_update = (Eigen::Matrix3d::Identity() - K_pos * H_pos) * P_pred;

    new_gps_position_ = false;
  }

  // GPS heading update
  if (new_gps_heading_) {
    double z_heading = gps_heading_meas_;
    double h_heading = state_update(2);
    double residual_heading = angleDiff(z_heading, h_heading);

    Eigen::Vector3d H_heading = Eigen::Vector3d::Zero();
    H_heading(2) = 1.0;

    double S_heading = H_heading.transpose() * P_update * H_heading + R_heading_;
    Eigen::Vector3d K_heading = P_update * H_heading / S_heading;

    state_update = state_update + K_heading * residual_heading;
    P_update = (Eigen::Matrix3d::Identity() - K_heading * H_heading.transpose()) * P_update;

    new_gps_heading_ = false;
  }

  // Update EKF state
  ekf_state_ = state_update;
  ekf_state_(2) = normalizeAngle(ekf_state_(2));
  ekf_P_ = P_update;

  // Publish odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "odom_utm";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = ekf_state_(0);
  odom_msg.pose.pose.position.y = ekf_state_(1);
  odom_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, ekf_state_(2));
  odom_msg.pose.pose.orientation = tf2::toMsg(q);

  // Set covariance
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = 0.0;
  }
  odom_msg.pose.covariance[0] = ekf_P_(0, 0);   // x
  odom_msg.pose.covariance[7] = ekf_P_(1, 1);   // y
  odom_msg.pose.covariance[35] = ekf_P_(2, 2);  // yaw

  odom_msg.twist.twist.linear.x = current_speed_ * cos(ekf_state_(2));
  odom_msg.twist.twist.linear.y = current_speed_ * sin(ekf_state_(2));
  odom_msg.twist.twist.angular.z = omega_z;

  odom_pub_ekf_->publish(odom_msg);
}

bool IMUGPSToOdometry::calculateGPSHeading(double &heading_rad)
{
  if (gps_utm_history_.size() < 2) {
    return false;
  }

  if (current_speed_ < min_speed_for_heading_) {
    return false;
  }

  auto newest = gps_utm_history_.back();
  auto oldest = gps_utm_history_.front();

  double dx = newest.first - oldest.first;
  double dy = newest.second - oldest.second;
  double distance = sqrt(dx * dx + dy * dy);

  if (distance < min_gps_distance_for_heading_) {
    return false;
  }

  heading_rad = atan2(dy, dx);
  return true;
}

void IMUGPSToOdometry::convertLatLonToUTM(double lat, double lon, double &easting, double &northing)
{
#ifdef HAS_GEOGRAPHICLIB
  int zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
#else
  // Simple approximation for UTM conversion without GeographicLib
  // This is a rough approximation and should be replaced with proper UTM conversion
  double lon_rad = lon * M_PI / 180.0;
  double lat_rad = lat * M_PI / 180.0;
  
  // Very rough approximation - not accurate for real use
  easting = lon_rad * 111320.0 * cos(lat_rad);
  northing = lat_rad * 111320.0;
  
  RCLCPP_WARN_ONCE(this->get_logger(), "Using approximate UTM conversion. Install GeographicLib for accurate conversion.");
#endif
}

double IMUGPSToOdometry::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double IMUGPSToOdometry::angleDiff(double a, double b)
{
  double diff = a - b;
  return normalizeAngle(diff);
}

double IMUGPSToOdometry::northDegreeToEastRadians(double degrees)
{
  return normalizeAngle((90.0 - degrees) * M_PI / 180.0);
}

geometry_msgs::msg::Quaternion IMUGPSToOdometry::northQuaternionToEastQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  
  double east_yaw = northDegreeToEastRadians(yaw * 180.0 / M_PI);
  
  tf2::Quaternion east_q;
  east_q.setRPY(roll, pitch, east_yaw);
  
  return tf2::toMsg(east_q);
}

void IMUGPSToOdometry::checkDataTimeouts()
{
  auto current_time = this->now();
  
  if ((current_time - last_gps_received_time_).seconds() > gps_timeout_threshold_) {
    gps_not_recv_count_++;
    if (gps_not_recv_count_ >= timeout_count_th_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GPS data timeout");
    }
  }
  
  if ((current_time - last_imu_received_time_).seconds() > imu_timeout_threshold_) {
    imu_not_recv_count_++;
    if (imu_not_recv_count_ >= timeout_count_th_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "IMU data timeout");
    }
  }
  
  if ((current_time - last_velocity_received_time_).seconds() > velocity_timeout_threshold_) {
    vel_not_recv_count_++;
    if (vel_not_recv_count_ >= timeout_count_th_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Velocity data timeout");
    }
  }
  
  all_data_callback_received_ = (gps_not_recv_count_ == 0) && 
                                (imu_not_recv_count_ == 0) && 
                                (vel_not_recv_count_ == 0);
}

}