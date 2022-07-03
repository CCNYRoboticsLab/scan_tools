cmake_minimum_required(VERSION 3.5)

project(ros2_laser_scan_matcher)

find_package(ament_cmake REQUIRED)
find_package(csm REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(laser_scan_matcher src/laser_scan_matcher.cpp)

target_include_directories(laser_scan_matcher
  PRIVATE
  "include"
)

ament_target_dependencies(laser_scan_matcher
  csm
  Eigen3
  geometry_msgs
  laser_geometry
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(TARGETS laser_scan_matcher DESTINATION lib/${PROJECT_NAME})

ament_package()