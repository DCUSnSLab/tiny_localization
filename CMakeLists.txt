cmake_minimum_required(VERSION 3.8)
project(tiny_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Find GeographicLib
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(tiny_localization_node
  src/main.cpp
  src/imugps_to_odometry.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(tiny_localization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link GeographicLib
target_link_libraries(tiny_localization_node ${GeographicLib_LIBRARIES})
target_compile_definitions(tiny_localization_node PRIVATE HAS_GEOGRAPHICLIB)

# Install executables
install(TARGETS
  tiny_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/gps_frame_broadcast.py
  scripts/odom_frame_broadcast.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
