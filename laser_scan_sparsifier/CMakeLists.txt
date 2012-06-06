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

# create laser_scan_sparsifier library

rosbuild_add_library (laser_scan_sparsifier src/laser_scan_sparsifier.cpp)

# create laser_scan_sparsifier_nodelet library

rosbuild_add_library (laser_scan_sparsifier_nodelet src/laser_scan_sparsifier_nodelet.cpp)
target_link_libraries (laser_scan_sparsifier_nodelet laser_scan_sparsifier)

# create laser_scan_sparsifier_node executable

rosbuild_add_executable(laser_scan_sparsifier_node src/laser_scan_sparsifier_node.cpp)
target_link_libraries (laser_scan_sparsifier_node laser_scan_sparsifier)
