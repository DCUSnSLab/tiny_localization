#include "imugps_to_odometry.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <GeographicLib/UTMUPS.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace tiny_localization {

IMUGPSToOdometry::IMUGPSToOdometry() :
  gps_fix_received_(false),
  init_position_flag_(true),
  last_time_(0.0),
  ekf_initialized_(false),
  gps_heading_meas_(0.0),
  new_gps_position_(false),
  new_gps_heading_(false),
  current_speed_(0.0),
  min_gps_distance_for_heading_(0.0),
  // Initialize timeout related variables
  gps_timeout_threshold_(1.0),
  imu_timeout_threshold_(0.5),
  velocity_timeout_threshold_(1.0)
{
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  // Topic settings
  std::string gps_fix_topic = "/ublox_gps/fix";
  std::string imu_topic = "/vectornav/IMU";
  std::string velocity_topic = "/vehicle/velocity";
  std::string output_odom_topic = "/odom/ekf";
  std::string output_utm_topic = "/current_utm_relative_position";

  if (nh.getParam("topics/gps_fix_topic", gps_fix_topic)) {
    ROS_INFO("Loaded gps_fix_topic: %s", gps_fix_topic.c_str());
  } else {
    ROS_WARN("Failed to get gps_fix_topic, using default: %s", gps_fix_topic.c_str());
  }

  if (nh.getParam("topics/imu_topic", imu_topic)) {
    ROS_INFO("Loaded imu_topic: %s", imu_topic.c_str());
  } else {
    ROS_WARN("Failed to get imu_topic, using default: %s", imu_topic.c_str());
  }

  if (nh.getParam("topics/velocity_topic", velocity_topic)) {
    ROS_INFO("Loaded velocity_topic: %s", velocity_topic.c_str());
  } else {
    ROS_WARN("Failed to get velocity_topic, using default: %s", velocity_topic.c_str());
  }

  if (nh.getParam("topics/output_odom_topic", output_odom_topic)) {
    ROS_INFO("Loaded output_odom_topic: %s", output_odom_topic.c_str());
  } else {
    ROS_WARN("Failed to get output_odom_topic, using default: %s", output_odom_topic.c_str());
  }

  if (nh.getParam("topics/output_utm_topic", output_utm_topic)) {
    ROS_INFO("Loaded output_utm_topic: %s", output_utm_topic.c_str());
  } else {
    ROS_WARN("Failed to get output_utm_topic, using default: %s", output_utm_topic.c_str());
  }

  // GPS heading settings
  min_gps_distance_for_heading_ = 0.0;  // default value
  if (nh.getParam("gps_heading/min_distance_for_heading", min_gps_distance_for_heading_)) {
    ROS_INFO("Loaded min_distance_for_heading: %f", min_gps_distance_for_heading_);
  } else {
    ROS_WARN("Failed to get min_distance_for_heading, using default: %f", min_gps_distance_for_heading_);
  }

  min_speed_for_heading_ = 0.6;  // default value
  if (nh.getParam("gps_heading/min_speed_for_heading", min_speed_for_heading_)) {
    ROS_INFO("Loaded min_speed_for_heading: %f", min_speed_for_heading_);
  } else {
    ROS_WARN("Failed to get min_speed_for_heading, using default: %f", min_speed_for_heading_);
  }

  // EKF process noise settings
  double process_noise_x = 0.05;  // default value
  double process_noise_y = 0.05;  // default value
  double process_noise_yaw = 0.001;  // default value

  if (nh.getParam("process_noise/x", process_noise_x)) {
    ROS_INFO("Loaded process_noise/x: %f", process_noise_x);
  } else {
    ROS_WARN("Failed to get process_noise/x, using default: %f", process_noise_x);
  }

  if (nh.getParam("process_noise/y", process_noise_y)) {
    ROS_INFO("Loaded process_noise/y: %f", process_noise_y);
  } else {
    ROS_WARN("Failed to get process_noise/y, using default: %f", process_noise_y);
  }

  if (nh.getParam("process_noise/yaw", process_noise_yaw)) {
    ROS_INFO("Loaded process_noise/yaw: %f", process_noise_yaw);
  } else {
    ROS_WARN("Failed to get process_noise/yaw, using default: %f", process_noise_yaw);
  }

  // EKF measurement noise settings
  double meas_noise_pos_x = 0.1;  // default value
  double meas_noise_pos_y = 0.1;  // default value
  double meas_noise_heading = 0.7;  // default value

  if (nh.getParam("measurement_noise/position/x", meas_noise_pos_x)) {
    ROS_INFO("Loaded measurement_noise/position/x: %f", meas_noise_pos_x);
  } else {
    ROS_WARN("Failed to get measurement_noise/position/x, using default: %f", meas_noise_pos_x);
  }

  if (nh.getParam("measurement_noise/position/y", meas_noise_pos_y)) {
    ROS_INFO("Loaded measurement_noise/position/y: %f", meas_noise_pos_y);
  } else {
    ROS_WARN("Failed to get measurement_noise/position/y, using default: %f", meas_noise_pos_y);
  }

  if (nh.getParam("measurement_noise/heading", meas_noise_heading)) {
    ROS_INFO("Loaded measurement_noise/heading: %f", meas_noise_heading);
  } else {
    ROS_WARN("Failed to get measurement_noise/heading, using default: %f", meas_noise_heading);
  }

  // Initial EKF settings
  double initial_pos_uncertainty = 1.0;  // default value
  double initial_heading_uncertainty = 1.0;  // default value

  if (nh.getParam("ekf_initial/position_uncertainty", initial_pos_uncertainty)) {
    ROS_INFO("Loaded ekf_initial/position_uncertainty: %f", initial_pos_uncertainty);
  } else {
    ROS_WARN("Failed to get ekf_initial/position_uncertainty, using default: %f", initial_pos_uncertainty);
  }

  if (nh.getParam("ekf_initial/heading_uncertainty", initial_heading_uncertainty)) {
    ROS_INFO("Loaded ekf_initial/heading_uncertainty: %f", initial_heading_uncertainty);
  } else {
    ROS_WARN("Failed to get ekf_initial/heading_uncertainty, using default: %f", initial_heading_uncertainty);
  }

  // Publishers and Subscribers setup
  current_utm_position_pub_ = nh_.advertise<geometry_msgs::Point>(output_utm_topic, 10);
  odom_pub_ekf_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic, 50);

  gps_fix_sub_ = nh_.subscribe(gps_fix_topic, 10, &IMUGPSToOdometry::gpsFixCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic, 50, &IMUGPSToOdometry::imuCallback, this);
  velocity_sub_ = nh_.subscribe(velocity_topic, 10, &IMUGPSToOdometry::velocityCallback, this);

  // Initialize EKF matrices
  Q_ = Eigen::Matrix3d::Zero();
  Q_(0,0) = process_noise_x;
  Q_(1,1) = process_noise_y;
  Q_(2,2) = process_noise_yaw;

  R_pos_ = Eigen::Matrix2d::Zero();
  R_pos_(0,0) = meas_noise_pos_x;
  R_pos_(1,1) = meas_noise_pos_y;
  
  R_heading_ = meas_noise_heading;

  ROS_INFO("IMUGPSToOdometry node (with EKF for position & heading) initialized.");

  // Setup timeout check timer (every 1 second)
  timeout_check_timer_ = nh_.createTimer(ros::Duration(1.0), 
                          [this](const ros::TimerEvent&) { this->checkDataTimeouts(); });
}

IMUGPSToOdometry::~IMUGPSToOdometry()
{
}

void IMUGPSToOdometry::spin()
{
  ros::spin();
}

// Callback functions

void IMUGPSToOdometry::gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  last_gps_received_time_ = ros::Time::now();
  gps_not_recv_count_ = 0;
  gps_fix_received_ = true;
  new_gps_position_ = true;
  
  gps_fix_msg_ = *msg;


  double easting, northing;
  convertLatLonToUTM(msg->latitude, msg->longitude, easting, northing);

  if (init_position_flag_) {
    ROS_INFO("Initializing position with first GPS fix...");
    ros::Duration(1.0).sleep(); // Wait for 1 second
    init_position_utm_.x = easting;
    init_position_utm_.y = northing;
    init_position_utm_.z = 0.0;
    std::ostringstream oss;
    oss << "{x: " << easting << ", y: " << northing << "}";
    ros::param::set("/init_position", oss.str());
    init_position_flag_ = false;
    ROS_INFO("Initial UTM position set.");
    

    gps_utm_history_.clear();
    gps_utm_history_.push_back(std::make_pair(easting, northing));
    last_gps_time_ = msg->header.stamp;
    return;
  }
  

  gps_utm_history_.push_back(std::make_pair(easting, northing));
  
  // Limit history size (maximum 20 points)
  if (gps_utm_history_.size() > 20) {
    gps_utm_history_.pop_front();
  }

  double heading_rad;

  if (calculateGPSHeading(heading_rad) && current_speed_ >= min_speed_for_heading_) {
    gps_heading_meas_ = heading_rad;
    new_gps_heading_ = true;
    ROS_DEBUG("New GPS heading calculated: %.2f degrees", heading_rad * 180.0 / M_PI);
  }

  new_gps_position_ = true;
  last_gps_time_ = msg->header.stamp;
}

void IMUGPSToOdometry::velocityCallback(const std_msgs::Float64::ConstPtr& msg)
{
  last_velocity_received_time_ = ros::Time::now();
  vel_not_recv_count_ = 0;
  current_speed_ = msg->data;
}

void IMUGPSToOdometry::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  last_imu_received_time_ = ros::Time::now();
  imu_not_recv_count_ = 0;
  imu_msg_ = *msg;
  double current_time = msg->header.stamp.toSec();

  if (init_position_flag_ || !gps_fix_received_ || !all_data_callback_received_) {
    return;
  }

  if (last_time_ == 0.0) {
    last_time_ = current_time;
    if (!ekf_initialized_) {
      ekf_state_ << 0.0, 0.0, 0.0;
      if (gps_heading_meas_ != 0.0) {
        ekf_state_(2) = gps_heading_meas_;
        ROS_INFO("EKF heading initialized with GPS measurement.");
      }
      
      // Load parameters directly here
      double pos_uncertainty = 1.0;
      double heading_uncertainty = 1.0;
      
      ros::NodeHandle nh;
      // Read from global namespace
      nh.getParam("/ekf_initial/position_uncertainty", pos_uncertainty);
      nh.getParam("/ekf_initial/heading_uncertainty", heading_uncertainty);
      
      ekf_P_ = Eigen::Matrix3d::Zero();
      ekf_P_(0,0) = pos_uncertainty;
      ekf_P_(1,1) = pos_uncertainty;
      ekf_P_(2,2) = heading_uncertainty;
      
      ekf_initialized_ = true;
    }
    return;
  }

  double dt = current_time - last_time_;
  last_time_ = current_time;

  if (!ekf_initialized_) {
    return;
  }

  double v = current_speed_;
  double omega_z = msg->angular_velocity.z;

  double yaw = ekf_state_(2);
  double predicted_x = ekf_state_(0) + v * cos(yaw) * dt;
  double predicted_y = ekf_state_(1) + v * sin(yaw) * dt;
  double predicted_yaw = normalizeAngle(yaw + omega_z * dt);

  Eigen::Vector3d X_pred;
  X_pred << predicted_x, predicted_y, predicted_yaw;

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0,2) = -v * sin(yaw) * dt;
  F(1,2) =  v * cos(yaw) * dt;

  Eigen::Matrix3d P_pred = F * ekf_P_ * F.transpose() + Q_;

  if (new_gps_position_) {
    double easting, northing;
    convertLatLonToUTM(gps_fix_msg_.latitude, gps_fix_msg_.longitude, easting, northing);
    double meas_x = easting - init_position_utm_.x;
    double meas_y = northing - init_position_utm_.y;

    Eigen::Vector2d Z_pos;
    Z_pos << meas_x, meas_y;

    Eigen::MatrixXd H_pos(2, 3);
    H_pos << 1, 0, 0,
             0, 1, 0;

    Eigen::Vector2d Z_pos_pred = H_pos * X_pred;

    Eigen::Vector2d Y_pos = Z_pos - Z_pos_pred;
    
    Eigen::Matrix2d S_pos = H_pos * P_pred * H_pos.transpose() + R_pos_;
    
    Eigen::MatrixXd K_pos = P_pred * H_pos.transpose() * S_pos.inverse();

    Eigen::Vector3d X_upd = X_pred + K_pos * Y_pos;
    
    Eigen::Matrix3d P_upd = (Eigen::Matrix3d::Identity() - K_pos * H_pos) * P_pred;

    X_pred = X_upd;
    P_pred = P_upd;

    new_gps_position_ = false;
  }

  if (new_gps_heading_) {
    double Z_hdg = gps_heading_meas_;
    Eigen::RowVector3d H_hdg;
    H_hdg << 0, 0, 1;

    double Z_hdg_pred = H_hdg * X_pred;

    double innov = angleDiff(Z_hdg, Z_hdg_pred);
    
    double S_hdg = H_hdg * P_pred * H_hdg.transpose() + R_heading_;
    Eigen::Vector3d K_hdg = P_pred * H_hdg.transpose() / S_hdg;

    Eigen::Vector3d X_upd = X_pred + K_hdg * innov;
    // yaw normalization
    X_upd(2) = normalizeAngle(X_upd(2));

    Eigen::Matrix3d P_upd = (Eigen::Matrix3d::Identity() - K_hdg * H_hdg) * P_pred;

    X_pred = X_upd;
    P_pred = P_upd;

    new_gps_heading_ = false;
  }

  ekf_state_ = X_pred;
  ekf_P_ = P_pred;

  // calculate odometry
  nav_msgs::Odometry ekf_odom;
  ekf_odom.header.stamp = msg->header.stamp;
  ekf_odom.header.frame_id = "odom_utm";
  ekf_odom.child_frame_id = "base_link";

  ekf_odom.pose.pose.position.x = ekf_state_(0);
  ekf_odom.pose.pose.position.y = ekf_state_(1);
  ekf_odom.pose.pose.position.z = 0.0;

  double yaw_ekf = ekf_state_(2);
  tf::Quaternion q_tf;
  q_tf.setRPY(0, 0, yaw_ekf);
  geometry_msgs::Quaternion q_msg;
  tf::quaternionTFToMsg(q_tf, q_msg);
  ekf_odom.pose.pose.orientation = q_msg;


  ekf_odom.twist.twist.linear.x = current_speed_;
  ekf_odom.twist.twist.angular.z = msg->angular_velocity.z;


  ekf_odom.pose.covariance[0] = ekf_P_(0,0);
  ekf_odom.pose.covariance[7] = ekf_P_(1,1);
  ekf_odom.pose.covariance[35] = ekf_P_(2,2);

  odom_pub_ekf_.publish(ekf_odom);

  // current utm position topic publish
  geometry_msgs::Point current_utm_relative_position;
  current_utm_relative_position.x = ekf_state_(0);
  current_utm_relative_position.y = ekf_state_(1);
  current_utm_relative_position.z = 0.0;
  current_utm_position_pub_.publish(current_utm_relative_position);
}

// calculate heading from gps position data
bool IMUGPSToOdometry::calculateGPSHeading(double &heading_rad)
{
  if (gps_utm_history_.size() < 2) {
    return false;
  }
  
  // calculate heading from two latest gps position
  const auto& newest = gps_utm_history_.back();
  
  // find previous gps point with enough distance
  for (int i = gps_utm_history_.size() - 2; i >= 0; i--) {
    const auto& older = gps_utm_history_[i];
    
    double dx = newest.first - older.first;   // East-West direction (easting)
    double dy = newest.second - older.second; // North-South direction (northing)
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // find previous gps point with enough distance
    if (distance >= min_gps_distance_for_heading_) {
      // atan2(dy, dx) returns east based heading
      heading_rad = std::atan2(dy, dx);
      
      // atan2 returns angle in the range of -π ~ π, so normalize it
      heading_rad = normalizeAngle(heading_rad);
      
      ROS_DEBUG("GPS Heading calculated from positions %.2f m apart", distance);
      return true;
    }
  }
  
  // not found enough distance
  return false;
}

// utility functions

double IMUGPSToOdometry::normalizeAngle(double angle)
{
  return atan2(sin(angle), cos(angle));
}

double IMUGPSToOdometry::angleDiff(double a, double b)
{
  double diff = a - b;
  return atan2(sin(diff), cos(diff));
}

double IMUGPSToOdometry::northDegreeToEastRadians(double degrees)
{
  double east_based_degrees = degrees - 90.0;
  return -east_based_degrees * M_PI / 180.0;
}

geometry_msgs::Quaternion IMUGPSToOdometry::northQuaternionToEastQuaternion(const geometry_msgs::Quaternion& q)
{
  tf::Quaternion q_current(q.x, q.y, q.z, q.w);
  tf::Quaternion q_rotate;
  q_rotate.setRPY(0, 0, M_PI / 2.0);
  tf::Quaternion q_new = q_rotate * q_current;
  q_new.normalize();
  geometry_msgs::Quaternion q_result;
  q_result.x = q_new.x();
  q_result.y = q_new.y();
  q_result.z = q_new.z();
  q_result.w = q_new.w();
  return q_result;
}

void IMUGPSToOdometry::convertLatLonToUTM(double lat, double lon, double &easting, double &northing)
{
  int zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
}

void IMUGPSToOdometry::checkDataTimeouts()
{
  gps_not_recv_count_++;
  imu_not_recv_count_++;
  vel_not_recv_count_++;
  if (vel_not_recv_count_ <= timeout_count_th_ && gps_not_recv_count_ <= timeout_count_th_ && imu_not_recv_count_ <= timeout_count_th_) {
    if (!all_data_callback_received_flag_) {
      ROS_INFO("All data callback received!");
      all_data_callback_received_flag_ = true;
      all_data_callback_received_ = true;
    }
    return;
  }

  if (gps_not_recv_count_ > timeout_count_th_) {
    ROS_WARN_THROTTLE(1.0, "No GPS data callback for %d seconds!", gps_not_recv_count_);
  }
  if (imu_not_recv_count_ > timeout_count_th_) {
    ROS_WARN_THROTTLE(1.0, "No IMU data callback for %d seconds!", imu_not_recv_count_);
  }
  if (vel_not_recv_count_ > timeout_count_th_) {
    ROS_WARN_THROTTLE(1.0, "No Velocity data callback for %d seconds!", vel_not_recv_count_);
  }
  all_data_callback_received_flag_ = false;
  all_data_callback_received_ = false;

}

}
