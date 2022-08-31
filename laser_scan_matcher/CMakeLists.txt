cmake_minimum_required(VERSION 3.5)
project(laser_scan_matcher)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)
link_directories(${csm_LIBRARY_DIRS})

include_directories(
  include
  ${csm_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

#Create library
add_library(${PROJECT_NAME}
  src/laser_scan_matcher.cpp
  src/param_handler.cpp)
target_link_libraries(${PROJECT_NAME} ${csm_LIBRARIES})

#Create node
add_executable(laser_scan_matcher_node  src/laser_scan_matcher_node.cpp)
target_link_libraries(laser_scan_matcher_node ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}
  csm
  Eigen3
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(
  TARGETS 
    ${PROJECT_NAME}
    laser_scan_matcher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
