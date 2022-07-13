cmake_minimum_required(VERSION 3.5)

project(laser_scan_matcher)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

message("csm_INCLUDE_DIRS: ${csm_INCLUDE_DIRS}")

include_directories(
  include
  ${csm_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

link_directories(${csm_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/laser_scan_matcher.cpp)
target_link_libraries(${PROJECT_NAME} ${csm_LIBRARIES})

target_include_directories(${PROJECT_NAME}
  PRIVATE
  "include"
)

ament_target_dependencies(${PROJECT_NAME}
  csm
  Eigen3
  geometry_msgs
  laser_geometry
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()