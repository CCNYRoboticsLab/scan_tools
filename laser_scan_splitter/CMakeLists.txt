cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# create laser_scan_splitter library

rosbuild_add_library (laser_scan_splitter src/laser_scan_splitter.cpp)

# create laser_scan_splitter_nodelet library

rosbuild_add_library (laser_scan_splitter_nodelet src/laser_scan_splitter_nodelet.cpp)
target_link_libraries (laser_scan_splitter_nodelet laser_scan_splitter)

# create laser_scan_splitter_node executable

rosbuild_add_executable(laser_scan_splitter_node src/laser_scan_splitter_node.cpp)
target_link_libraries (laser_scan_splitter_node laser_scan_splitter)
