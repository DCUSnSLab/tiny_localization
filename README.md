# Tiny Localization

A lightweight Extended Kalman Filter (EKF) based localization package for autonomous vehicles, fusing GPS and IMU data to estimate vehicle position and heading.

## Overview

<p align="center">
  <img src="https://github.com/user-attachments/assets/efa71c82-fe60-4fd9-a0c5-c19b8350b13b" alt="Image" />
</p>

Tiny Localization provides a simple yet effective solution for estimating a vehicle's global position and orientation using sensor fusion. 

## Enviroment
- Ubuntu 20.04
- ROS Noetic


## Node Input/Output
![Image](https://github.com/user-attachments/assets/ee42a863-6dbe-4176-a4e2-93d83fe56c27)

## TF Tree
![Image](https://github.com/user-attachments/assets/4e9884ff-ca17-4e29-aac5-58b5479558f2)


## Architecture
![Image](https://github.com/user-attachments/assets/98b04965-83ef-4cba-933e-73ad0bddd966)


## Features

- Fusion of GPS, IMU, and vehicle speed data for robust position and heading estimation
- GPS-based heading calculation with configurable parameters
- East-based coordinate system with UTM conversion
- Fully configurable EKF parameters via YAML configuration file
- Standard ROS odometry output for integration with navigation packages
- Support for forward and reverse motion

## Dependencies
- GeographicLib (for UTM conversions)
- Eigen3 (for matrix operations)

## Installation

1. Clone this repository into your catkin workspace's `src` directory:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/totwjfakd/tiny_localization.git
   ```

2. Install dependencies:
   ```bash
   sudo apt-get install ros-noetic-geographic-msgs libgeographic-dev
   ```

3. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## Usage

1. Configure parameters in `cfg/config.yaml` according to your setup
2. Launch the node:
   ```bash
   roslaunch tiny_localization tiny_localization.launch
   ```
3. No TF publish Launch:
   ```bash
   roslaunch tiny_localization tiny_localization.launch tf_broadcast_enabled:=false
   ```
## Configuration

All parameters can be adjusted in the `cfg/config.yaml` file:

### Topic Settings
```yaml
topics: 
  gps_fix_topic: "/ublox_gps/fix"       # GPS fix topic from ublox receiver  (sensor_msgs/NavSatFix)
  imu_topic: "/vectornav/IMU"           # IMU topic from VectorNav sensor (sensor_msgs/Imu)
  velocity_topic: "/vehicle/velocity"    # Vehicle velocity topic (std_msgs/Float64)
  output_odom_topic: "/odom/ekf"  # Output odometry topic (nav_msgs/Odometry)
  output_utm_topic: "/current_utm_relative_position"  # Output UTM position topic (geometry_msgs/Point)
```

### GPS Heading Calculation Settings
```yaml
gps_heading:
  min_distance_for_heading: 1.0  # Minimum GPS distance threshold for heading calculation (meters)
  min_speed_for_heading: 0.6     # Minimum speed threshold for reliable heading calculation (m/s)
```

### EKF Process Noise Settings (Q Matrix)
```yaml
process_noise:
  x: 0.05    # Process noise for x position
  y: 0.05    # Process noise for y position
  yaw: 0.001 # Process noise for heading/yaw
```

### EKF Measurement Noise Settings (R Matrix)
```yaml
measurement_noise:
  position:
    x: 0.1    # Measurement noise for x position
    y: 0.1    # Measurement noise for y position
  heading: 0.7 # Measurement noise for heading/yaw
```

### Initial EKF Settings
```yaml
ekf_initial:
  position_uncertainty: 1.0   # Initial position uncertainty
  heading_uncertainty: 1.0    # Initial heading uncertainty
```

## Subscribed Topics

- `gps_fix_topic` (sensor_msgs/NavSatFix): GPS fix data
- `imu_topic` (sensor_msgs/Imu): IMU data with angular velocities
- `velocity_topic` (std_msgs/Float64): Vehicle speed (m/s)

## Published Topics

- `/odom/ekf` (nav_msgs/Odometry): Estimated vehicle pose and twist
- `/current_utm_relative_position` (geometry_msgs/Point): Current position in local UTM coordinates

## Coordinate System

The localization system uses an East-North-Up (ENU) coordinate system:
- X-axis points East
- Y-axis points North
- Heading is measured from the East axis in the counter-clockwise direction

## Algorithm Overview

1. The system initializes with the first GPS position as the origin of the local coordinate system. The GPS coordinates are converted to UTM to create a metric-based local reference frame. (This UTM initial position is available from the ROS Parameter Server at '/init_position')
2. EKF prediction is performed using the motion model based on vehicle speed and IMU angular velocity
3. GPS position measurements update the position states
4. GPS-derived heading is calculated from consecutive positions and updates the heading state
5. The final state is published as ROS odometry messages

## Known Issues and Limitations

- Heading calculation may be unstable during transitions between forward and reverse motion
- Requires reliable GPS reception for accurate positioning
- Initial position uncertainty can be high until sufficient movement is detected
- The algorithm assumes planar motion (2D movement)

## License

This package is released under the MIT License. See LICENSE file for details.
