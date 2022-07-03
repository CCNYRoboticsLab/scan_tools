cmake_minimum_required(VERSION 3.5)

project(scan_to_cloud_converter)

find_package(ament_cmake REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(${PROJECT_NAME} src/scan_to_cloud_converter.cpp)
target_include_directories(${PROJECT_NAME}
  PRIVATE
  "include"
)
ament_target_dependencies(${PROJECT_NAME}
  laser_geometry
  rclcpp
  sensor_msgs
  tf2
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
