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

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <csm/csm.h>  // csm defines min and max, but Eigen complains

namespace scan_tools {

class LaserScanMatcher: public rclcpp::Node
{
public:
  LaserScanMatcher();
  ~LaserScanMatcher() = default;

private:
  // Ros handle

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;

  // bookkeeping
  bool is_running_;

  // Coordinate parameters
  std::string base_frame_ = "base_link";
  std::string odom_frame_ = "odom";

  // Keyframe parameters
  double min_travel_distance_ = 0.5;
  double min_travel_heading_ = 30.0;

  bool initialized_ = false;
  bool publish_tf_ = false;
  bool degeneracy_check_ = false;

  double tf_timeout_ = 0.1;
  double xy_cov_scale_ = 1.0;
  double xy_cov_offset_ = 0.0;
  double heading_cov_scale_ = 1.0;
  double heading_cov_offset_ = 0.0;
  double degeneracy_cov_ramp_ = 5.0;
  double degeneracy_cov_scale_ = 1.0;
  double degeneracy_cov_offset_ = 0.0;
  double degeneracy_threshold_ = 1.0;

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

  std::vector<double> a_cos_;
  std::vector<double> a_sin_;

  rclcpp::Time prev_stamp_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::unordered_map<std::string, bool*> bool_params_;
  std::unordered_map<std::string, double*> double_params_;
  std::unordered_map<std::string, int*> int_params_;
  std::unordered_map<std::string, std::string*> string_params_;

  template <class T>
  T param(const std::string& name, const T& default_val, const std::string& description) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = true;
    declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    return get_parameter(name).get_value<T>();
  }

  template <class T>
  void register_param(T* param, const std::string& name, const T& default_val, const std::string& description) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    auto p = get_parameter(name);
    *param = p.get_value<T>();

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      bool_params_[name] = static_cast<bool*>(static_cast<void*>(param));
    }
    else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double_params_[name] = static_cast<double*>(static_cast<void*>(param));
    }
    else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      int_params_[name] = static_cast<int*>(static_cast<void*>(param));
    }
    else {
      RCLCPP_ERROR(get_logger(), "Unsupported dynamic parameter type: %s for parameter: %s", p.get_type_name().c_str(), name.c_str());
    }
  }

  void register_param(int* param, const std::string& name, bool default_val, const std::string& description) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    *param = get_parameter(name).as_bool();
    int_params_[name] = param;
  }

  void register_param(int* param, const std::string& name, int default_val, const std::string& description, int min, int max) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = min;
    descriptor.integer_range[0].to_value = max;
    descriptor.integer_range[0].step = 0;
    declare_parameter(name, default_val, descriptor);
    *param = get_parameter(name).as_int();
    int_params_[name] = param;
  }

  void register_param(double* param, const std::string& name, double default_val, const std::string& description, double min, double max) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = min;
    descriptor.floating_point_range[0].to_value = max;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter(name, default_val, descriptor);
    *param = get_parameter(name).as_double();
    double_params_[name] = param;
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  bool getBaseToLaserTf (const std::string& frame_id);
  bool getLaserInTfOdom(const std::string& frame_id, const rclcpp::Time& stamp, tf2::Transform& transform);
  Eigen::Matrix2f getLaserRotation(const tf2::Transform& odom_pose);

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  bool processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, const tf2::Transform& pred_laser_offset);
  void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan, LDP& ldp);
  void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);

  void start();
  void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  void stop();
  void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  bool newKeyframeNeeded(const tf2::Transform& d);
  void publishKeyframe(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  void createCache(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg);

  Eigen::Vector2f checkAxisDegeneracy(const laser_data& scan, float baseline, const std::string& laser_frame, rclcpp::Time stamp);

};  // LaserScanMatcher

}  // namespace scan_tools
