#include "tiny_localization/imugps_to_odometry.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <GeographicLib/UTMUPS.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace my_odometry_package {

IMUGPSToOdometry::IMUGPSToOdometry() :
  gps_fix_received_(false),
  init_position_flag_(true),
  last_time_(0.0),
  ekf_initialized_(false),
  gps_heading_meas_(0.0),
  new_gps_position_(false),
  new_gps_heading_(false),
  current_speed_(0.0),
  min_gps_distance_for_heading_(0.0)
{
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  std::string gps_fix_topic, imu_topic, velocity_topic;
  nh.param<std::string>("topics/gps_fix_topic", gps_fix_topic, "/ublox_gps/fix");
  nh.param<std::string>("topics/imu_topic", imu_topic, "/vectornav/IMU");
  nh.param<std::string>("topics/velocity_topic", velocity_topic, "/vehicle/velocity");
  
  // 출력 토픽 이름을 매개변수로 가져오기
  std::string output_odom_topic, output_utm_topic;
  nh.param<std::string>("topics/output_odom_topic", output_odom_topic, "/odom/ekf_single");
  nh.param<std::string>("topics/output_utm_topic", output_utm_topic, "/current_utm_relative_position");
  
  // 방위각 계산 관련 설정 - 전역 네임스페이스에서 직접 가져오기
  double min_distance_for_heading;
  if (nh.getParam("/gps_heading/min_distance_for_heading", min_distance_for_heading)) {
    min_gps_distance_for_heading_ = min_distance_for_heading;
    ROS_INFO("Loaded min_distance_for_heading from parameter server: %f", min_gps_distance_for_heading_);
  } else {
    ROS_WARN("Failed to get min_distance_for_heading from parameter server, using default (0.0)");
  }
  
  min_speed_for_heading_ = 0.6; // default value
  if (nh.getParam("/gps_heading/min_speed_for_heading", min_speed_for_heading_)) {
    ROS_INFO("Loaded min_speed_for_heading from parameter server: %f", min_speed_for_heading_);
  } else {
    ROS_WARN("Failed to get min_speed_for_heading from parameter server, using default (0.7)");
  }

  // EKF 프로세스 노이즈 (Q 행렬)
  double process_noise_x = 0.05;
  double process_noise_y = 0.05;
  double process_noise_yaw = 0.001;
  
  if (nh.getParam("/process_noise/x", process_noise_x)) {
    ROS_INFO("Loaded process_noise/x: %f", process_noise_x);
  }
  if (nh.getParam("/process_noise/y", process_noise_y)) {
    ROS_INFO("Loaded process_noise/y: %f", process_noise_y);
  }
  if (nh.getParam("/process_noise/yaw", process_noise_yaw)) {
    ROS_INFO("Loaded process_noise/yaw: %f", process_noise_yaw);
  }

  // EKF 측정 노이즈 (R 행렬)
  double meas_noise_pos_x = 0.1;
  double meas_noise_pos_y = 0.1;
  double meas_noise_heading = 0.7;
  
  if (nh.getParam("/measurement_noise/position/x", meas_noise_pos_x)) {
    ROS_INFO("Loaded measurement_noise/position/x: %f", meas_noise_pos_x);
  }
  if (nh.getParam("/measurement_noise/position/y", meas_noise_pos_y)) {
    ROS_INFO("Loaded measurement_noise/position/y: %f", meas_noise_pos_y);
  }
  if (nh.getParam("/measurement_noise/heading", meas_noise_heading)) {
    ROS_INFO("Loaded measurement_noise/heading: %f", meas_noise_heading);
  }

  // 초기 EKF 설정
  double initial_pos_uncertainty = 1.0;
  double initial_heading_uncertainty = 1.0;
  
  if (nh.getParam("/ekf_initial/position_uncertainty", initial_pos_uncertainty)) {
    ROS_INFO("Loaded ekf_initial/position_uncertainty: %f", initial_pos_uncertainty);
  }
  if (nh.getParam("/ekf_initial/heading_uncertainty", initial_heading_uncertainty)) {
    ROS_INFO("Loaded ekf_initial/heading_uncertainty: %f", initial_heading_uncertainty);
  }

  // print settings
  ROS_INFO_STREAM("=== Topic Settings ===");
  ROS_INFO_STREAM("GPS Fix Topic: " << gps_fix_topic);
  ROS_INFO_STREAM("IMU Topic: " << imu_topic);
  ROS_INFO_STREAM("Velocity Topic: " << velocity_topic);
  ROS_INFO_STREAM("Output Odometry Topic: " << output_odom_topic);
  ROS_INFO_STREAM("Output UTM Position Topic: " << output_utm_topic);
  
  ROS_INFO_STREAM("=== GPS Heading Settings ===");
  ROS_INFO_STREAM("Minimum Distance for Heading Calculation: " << min_gps_distance_for_heading_ << " m");
  ROS_INFO_STREAM("Minimum Speed for Heading Calculation: " << min_speed_for_heading_ << " m/s");
  
  ROS_INFO_STREAM("=== EKF Noise Settings ===");
  ROS_INFO_STREAM("Process Noise - X: " << process_noise_x << ", Y: " << process_noise_y << ", Yaw: " << process_noise_yaw);
  ROS_INFO_STREAM("Measurement Noise - Position X: " << meas_noise_pos_x << ", Position Y: " << meas_noise_pos_y << ", Heading: " << meas_noise_heading);
  
  ROS_INFO_STREAM("=== Initial EKF Settings ===");
  ROS_INFO_STREAM("Initial Position Uncertainty: " << initial_pos_uncertainty);
  ROS_INFO_STREAM("Initial Heading Uncertainty: " << initial_heading_uncertainty);

  current_utm_position_pub_ = nh_.advertise<geometry_msgs::Point>(output_utm_topic, 10);
  odom_pub_ekf_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic, 50);


  gps_fix_sub_ = nh_.subscribe(gps_fix_topic, 10, &IMUGPSToOdometry::gpsFixCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic, 50, &IMUGPSToOdometry::imuCallback, this);
  velocity_sub_ = nh_.subscribe(velocity_topic, 10, &IMUGPSToOdometry::velocityCallback, this);


  Q_ = Eigen::Matrix3d::Zero();
  Q_(0,0) = process_noise_x;
  Q_(1,1) = process_noise_y;
  Q_(2,2) = process_noise_yaw;

  R_pos_ = Eigen::Matrix2d::Zero();
  R_pos_(0,0) = meas_noise_pos_x;
  R_pos_(1,1) = meas_noise_pos_y;
  
  R_heading_ = meas_noise_heading;

  ROS_INFO("IMUGPSToOdometry node (with EKF for position & heading) initialized.");
}

IMUGPSToOdometry::~IMUGPSToOdometry()
{
}

void IMUGPSToOdometry::spin()
{
  ros::spin();
}

// callback 함수들

void IMUGPSToOdometry::gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_fix_msg_ = *msg;
  gps_fix_received_ = true;

  double easting, northing;
  convertLatLonToUTM(msg->latitude, msg->longitude, easting, northing);

  if (init_position_flag_) {
    ROS_INFO("Initializing position with first GPS fix...");
    ros::Duration(1.0).sleep(); // 1초 대기
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
  
  // 히스토리 크기 제한 (최대 20개 포인트 유지)
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
  current_speed_ = msg->data;
}

void IMUGPSToOdometry::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msg_ = *msg;
  double current_time = msg->header.stamp.toSec();

  if (init_position_flag_ || !gps_fix_received_) {
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
      
      // 여기에서도 파라미터를 직접 로드
      double pos_uncertainty = 1.0;
      double heading_uncertainty = 1.0;
      
      ros::NodeHandle nh;
      // 전역 네임스페이스에서 읽기
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
  q_tf.setRPY(0, 0, yaw_ekf);  // roll=0, pitch=0, yaw(east based)
  geometry_msgs::Quaternion q_msg;
  tf::quaternionTFToMsg(q_tf, q_msg);
  ekf_odom.pose.pose.orientation = q_msg;


  ekf_odom.twist.twist.linear.x = current_speed_;
  ekf_odom.twist.twist.angular.z = msg->angular_velocity.z;


  ekf_odom.pose.covariance[0] = ekf_P_(0,0);   // x
  ekf_odom.pose.covariance[7] = ekf_P_(1,1);   // y
  ekf_odom.pose.covariance[35] = ekf_P_(2,2);  // yaw

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
    
    double dx = newest.first - older.first;   // 동서 방향 (easting)
    double dy = newest.second - older.second; // 남북 방향 (northing)
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

}
