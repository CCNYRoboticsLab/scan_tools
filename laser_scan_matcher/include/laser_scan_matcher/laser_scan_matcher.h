/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <csm/csm.h>  // csm defines min and max, but Eigen complains

namespace scan_tools {

class LaserScanMatcher: public rclcpp::Node
{
public:
  LaserScanMatcher();
  ~LaserScanMatcher();

private:
  // Ros handle

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfB_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;


  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub_;

  // Coordinate parameters
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string laser_frame_;
  std::string odom_topic_;

  // Keyframe parameters
  double kf_dist_linear_;
  double kf_dist_linear_sq_;
  double kf_dist_angular_;

  bool initialized_;
  bool publish_tf_;

  double xy_cov_scale_ = 1.0;
  double xy_cov_offset_ = 0.0;
  double heading_cov_scale_ = 1.0;
  double heading_cov_offset_ = 0.0;

  tf2::Transform base_from_laser_;  // static, cached
  tf2::Transform laser_from_base_;
  tf2::Transform prev_laser_in_tf_odom_;

  tf2::Transform base_in_fixed_;           // fixed-to-base tf (pose of base frame in fixed frame)
  tf2::Transform prev_base_in_fixed_;      // previous fixed-to-base tf (for odometry calculation)
  tf2::Transform keyframe_base_in_fixed_;  // pose of the base in fixed frame when last keyframe is taken

  sm_params input_;
  sm_result output_;
  LDP keyframe_laser_data_;

  laser_geometry::LaserProjection laser_projector_;

  // Grid map parameters
  double resolution_;

  std::vector<double> a_cos_;
  std::vector<double> a_sin_;

  rclcpp::Time prev_stamp_;

  bool getBaseToLaserTf (const std::string& frame_id);
  bool getLaserInTfOdom(const std::string& frame_id, const rclcpp::Time& stamp, tf2::Transform& transform);

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  bool processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, const tf2::Transform& pred_laser_offset);
  void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan, LDP& ldp);
  void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);

  bool newKeyframeNeeded(const tf2::Transform& d);

  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false);
  void createCache (const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg);


};  // LaserScanMatcher

}  // namespace scan_tools
