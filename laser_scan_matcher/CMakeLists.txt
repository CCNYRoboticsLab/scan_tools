cmake_minimum_required(VERSION 3.10.2)
project(laser_scan_matcher)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  # pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
  Boost)

# Find all required ROS2 packages
find_package(ament_cmake REQUIRED)
foreach(DEPENDENCY ${ROS_CXX_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html
set(FPHSA_NAME_MISMATCHED 1)
find_package(PCL REQUIRED QUIET)
unset(FPHSA_NAME_MISMATCHED)

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

# Set include directories
include_directories(include ${csm_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

#Create library
add_library(laser_scan_matcher SHARED src/laser_scan_matcher.cpp)

#Note we don't link against pcl as we're using header-only parts of the library
ament_target_dependencies(laser_scan_matcher
  tf2
  tf2_ros
  Boost
  pcl_conversions
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp)
target_link_libraries( laser_scan_matcher ${csm_LIBRARIES})

#Create component
add_library(laser_scan_matcher_component SHARED src/laser_scan_matcher_component.cpp)
target_link_libraries(laser_scan_matcher_component laser_scan_matcher)
rclcpp_components_register_nodes(laser_scan_matcher_component "scan_tools::LaserScanMatcherComponent")

#Create node
add_executable(laser_scan_matcher_node src/laser_scan_matcher_node.cpp)
target_link_libraries(laser_scan_matcher_node laser_scan_matcher)

#Install library
install(TARGETS laser_scan_matcher laser_scan_matcher_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

#Install library includes
install(DIRECTORY include/laser_scan_matcher/
    DESTINATION include)

#Install node
install(TARGETS laser_scan_matcher_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})

#Install demo files
install(DIRECTORY demo
    DESTINATION share/${PROJECT_NAME})

# add_rostest(test/run.test)
# add_rostest(test/covariance.test)
ament_package()