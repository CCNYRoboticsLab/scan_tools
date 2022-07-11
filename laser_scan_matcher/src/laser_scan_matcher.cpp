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

/*  This package uses Canonical Scan Matcher [1], written by Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#include <laser_scan_matcher/laser_scan_matcher.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#undef min
#undef max

namespace scan_tools {

LaserScanMatcher::LaserScanMatcher() : rclcpp::Node("laser_scan_matcher"), initialized_(false) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());

  // static parameters
  base_frame_ = param("base_frame", std::string("base_link"), "Which frame to use for the robot base");
  odom_frame_ = param("odom_frame", std::string("odom"), "Which frame to track odometry in");
  publish_tf_,  param("publish_tf", true, "Whether to publish tf transform from 'odom_frame' to 'base_frame'");

  // dynamic parameters
  register_param(&tf_timeout_, "tf_timeout", 0.1, "TF timeout in seconds.", 0.0, 10.0);
  register_param(&degeneracy_check_, "degeneracy_check", false, "Check for degeneracy of matches along an axis");
  register_param(&degeneracy_cov_ramp_, "degeneracy_cov_ramp", 5.0, "Power to apply to [0,1] degeneracy metric prior to scaling for covariance.", 1.0, 10.0);
  register_param(&degeneracy_cov_scale_, "degeneracy_cov_scale", 1.0, "Scaling degeneracy metric to apply to position covariance along degenerate axis", 0.0, 100.0);
  register_param(&degeneracy_cov_offset_, "degeneracy_cov_offset", 0.0, "Offset to apply to position covariance along degenerate axis", 0.0, 10.0);
  register_param(&xy_cov_scale_, "xy_cov_scale", 1.0, "Scaling to apply to xy position covariance", 0.0, 1e8);
  register_param(&xy_cov_offset_, "xy_cov_offset", 0.0, "Offset to apply to xy position covariance", 0.0, 10.0);
  register_param(&heading_cov_scale_, "heading_cov_scale", 1.0, "Scaling to apply to heading covariance", 0.0, 1e8);
  register_param(&heading_cov_offset_, "heading_cov_offset", 0.0, "Offset to apply to heading covariance", 0.0, 10.0);
  register_param(&min_travel_distance_, "min_travel_distance", 0.5, "Distance in meters to trigger a new keyframe", 0.0, 10.0);
  register_param(&min_travel_heading_, "min_travel_heading", 30.0, "Angle in degrees to trigger a new keyframe.", 0.0, 180.0);

  // CSM parameters - comments copied from algos.h (by Andrea Censi)
  register_param(&input_.max_angular_correction_deg, "max_angular_correction_deg", 45.0, "Maximum angular displacement between scans.", 0.0, 90.0);
  register_param(&input_.max_linear_correction, "max_linear_correction", 0.5, "Maximum translation between scans (m).", 0.0, 10.0);
  register_param(&input_.max_iterations, "max_iterations", 10, "Maximum ICP cycle iterations", 1, 100);
  register_param(&input_.epsilon_xy, "epsilon_xy", 1e-6, "A threshold for stopping (m).", 1e-10, 1e-1);
  register_param(&input_.epsilon_theta, "epsilon_theta", 1e-6, "A threshold for stopping (rad).", 1e-10, 1e-1);
  register_param(&input_.max_correspondence_dist, "max_correspondence_dist", 0.3, "Maximum distance for a correspondence to be valid.", 0.0, 10.0);
  register_param(&input_.sigma, "sigma", 0.01, "Noise in the scan (m).", 0.0, 1.0);
  register_param(&input_.use_corr_tricks, "use_corr_tricks", true, "Use smart tricks for finding correspondences.");
  register_param(&input_.restart, "restart", false, "Restart if error is over threshold.");
  register_param(&input_.restart_threshold_mean_error, "restart_threshold_mean_error", 0.01, "Threshold for restarting.", 0.0, 1.0);
  register_param(&input_.restart_dt, "restart_dt", 1.0, "Displacement for restarting. (m).", 0.0, 10.0);
  register_param(&input_.restart_dtheta, "restart_dtheta", 0.1, "Displacement for restarting. (rad).", 0.0, M_PI_2);
  register_param(&input_.clustering_threshold, "clustering_threshold", 0.25, "Max distance for staying in the same clustering.", 0.0, 1.0);
  register_param(&input_.orientation_neighbourhood, "orientation_neighbourhood", 20, "Number of neighbour rays used to estimate the orientation.", 0, 100);
  register_param(&input_.use_point_to_line_distance, "use_point_to_line_distance", true, "If false, it's vanilla ICP.");
  register_param(&input_.do_alpha_test, "do_alpha_test", false, "Discard correspondences based on the angles.");
  register_param(&input_.do_alpha_test_thresholdDeg, "alpha_test_threshold", 20.0, "Threshold angle to discard correspondences, in degrees.", 0.0, 90.0);
  register_param(&input_.outliers_maxPerc, "outliers_max_perc", 0.9, "Percentage [0, 1] of correspondences to consider with lowest error", 0.0, 1.0);
  register_param(&input_.outliers_adaptive_order, "outliers_adaptive_order", 0.7, "Percentile [0, 1] to use for adaptive outlier detection", 0.0, 1.0);
  register_param(&input_.outliers_adaptive_mult, "outliers_adaptive_mult", 2.0, "Threshold multiplier for error at chosen percentile", 0.0, 10.0);
  register_param(&input_.do_visibility_test, "do_visibility_test", false, "Use visibility test trick for matching.");
  register_param(&input_.outliers_remove_doubles, "outliers_remove_doubles", true, "Do not allow two different correspondences to share a point");
  register_param(&input_.do_compute_covariance, "do_compute_covariance", true, "Compute the covariance of the ICP");
  register_param(&input_.debug_verify_tricks, "debug_verify_tricks", false, "Checks that find_correspondences_tricks gives the right answer.");
  register_param(&input_.use_ml_weights, "use_ml_weights", false, "Use the 'true_alpha' or 'alpha' field to weight the impact of each correspondence");
  register_param(&input_.use_sigma_weights, "use_sigma_weights", false, "Use the 'readings_sigma' field of the second scan to weight the correspondence.");

  params_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&LaserScanMatcher::parametersCallback, this, std::placeholders::_1));

  // state variables
  base_in_fixed_.setIdentity();
  prev_base_in_fixed_.setIdentity();
  keyframe_base_in_fixed_.setIdentity();
  prev_laser_in_tf_odom_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;
  output_.cov_x_m = nullptr;
  output_.dx_dy1_m = nullptr;
  output_.dx_dy2_m = nullptr;

  // publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("laser_odom", rclcpp::SystemDefaultsQoS());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("laser_pose", rclcpp::SystemDefaultsQoS());
  keyframe_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", rclcpp::SystemDefaultsQoS());
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  // subscribers
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanMatcher::scanCallback, this, std::placeholders::_1));
  tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

rcl_interfaces::msg::SetParametersResult LaserScanMatcher::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param: parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      auto bool_param = bool_params_.find(param.get_name());
      if (bool_param != bool_params_.end()) {
        *bool_param->second = param.as_bool();
        RCLCPP_INFO(get_logger(), "Parameter updated: %s = %s", param.get_name().c_str(), (param.as_bool() ? "true": "false"));
      }
      else {
        auto int_param = int_params_.find(param.get_name());
        if (int_param != int_params_.end()) {
          *int_param->second = param.as_bool();
          RCLCPP_INFO(get_logger(), "Parameter updated: %s = %s", param.get_name().c_str(), (param.as_bool() ? "true": "false"));
        }
        RCLCPP_WARN(get_logger(), "Unknown bool parameter updated: %s", param.get_name().c_str());
      }
    }
    else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      auto double_param = double_params_.find(param.get_name());
      if (double_param != double_params_.end()) {
        *double_param->second = param.as_double();
        RCLCPP_INFO(get_logger(), "Parameter updated: %s = %lf", param.get_name().c_str(), param.as_double());
      }
      else {
        RCLCPP_WARN(get_logger(), "Unknown double parameter updated: %s", param.get_name().c_str());
      }
    }
    else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      auto int_param = int_params_.find(param.get_name());
      if (int_param != int_params_.end()) {
        *int_param->second = param.as_int();
        RCLCPP_INFO(get_logger(), "Parameter updated: %s = %ld", param.get_name().c_str(), param.as_int());
      }
      else {
        RCLCPP_WARN(get_logger(), "Unknown int parameter updated: %s", param.get_name().c_str());
      }
    }
    else {
      RCLCPP_WARN(get_logger(), "Unsupported parameter type updated: %s (%s)", param.get_name().c_str(), param.get_type_name().c_str());
    }
  }

  return result;
}

void LaserScanMatcher::createCache(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg) {
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i) {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}


void LaserScanMatcher::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  tf2::Transform laser_in_tf_odom;
  if (!getLaserInTfOdom(scan_msg->header.frame_id, scan_msg->header.stamp, laser_in_tf_odom)) {
    return;
  }

  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      return;
    }

    laserScanToLDP(scan_msg, keyframe_laser_data_);
    prev_stamp_ = scan_msg->header.stamp;
    prev_laser_in_tf_odom_ = laser_in_tf_odom;
    initialized_ = true;
  }

  auto pred_laser_offset = prev_laser_in_tf_odom_.inverse() * laser_in_tf_odom;

  if (processScan(scan_msg, pred_laser_offset)) {
    prev_laser_in_tf_odom_ = laser_in_tf_odom;
  }
  else {
    // TODO(malban): need to reset at some point?
    RCLCPP_WARN(get_logger(), "  failed to process scan");
  }

  prev_stamp_ = scan_msg->header.stamp;
}

bool LaserScanMatcher::getBaseToLaserTf(const std::string& frame_id) {
  try {
      auto msg = tf_buffer_->lookupTransform(base_frame_, frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(tf_timeout_));
      tf2::fromMsg(msg.transform, base_from_laser_);
      laser_from_base_ = base_from_laser_.inverse();
  }
  catch (tf2::TransformException ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Could not get initial transform of base to laser frame, %s", ex.what());
    return false;
  }

  return true;
}

bool LaserScanMatcher::getLaserInTfOdom(
  const std::string& frame_id,
  const rclcpp::Time& stamp,
  tf2::Transform& transform)
{
  if (!tf_buffer_->_frameExists(odom_frame_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf frame [%s] doesn't exist yet.'", odom_frame_.c_str());
    return false;
  }

  if (!tf_buffer_->_frameExists(frame_id)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf frame [%s] doesn't exist yet.'", frame_id.c_str());
    return false;
  }

  try {
    auto msg = tf_buffer_->lookupTransform(odom_frame_, frame_id, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
    tf2::fromMsg(msg.transform, transform);
  }
  catch (tf2::TransformException ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Could not get transform laser to fixed frame, %s", ex.what());
    return false;
  }

  return true;
}

bool LaserScanMatcher::processScan(
  const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
  const tf2::Transform& pred_laser_offset)
{
  RCLCPP_DEBUG(get_logger(),"Processing scan ...");

  // Canonical Scan Matcher (CSM) is used in the following way:
  //  - The scans are always in the laser frame.
  //  - The reference scan (keyframe_laser_data_) has a pose of [0, 0, 0]
  //  - The new scan (curr_laser_data) has a pose equal to the movement of the
  //      laser in the laser frame since the last scan
  //  - The computed correction is then propagated using the tf machinery

  LDP curr_laser_data;
  laserScanToLDP(scan_msg, curr_laser_data);

  keyframe_laser_data_->odometry[0] = 0.0;
  keyframe_laser_data_->odometry[1] = 0.0;
  keyframe_laser_data_->odometry[2] = 0.0;

  keyframe_laser_data_->estimate[0] = 0.0;
  keyframe_laser_data_->estimate[1] = 0.0;
  keyframe_laser_data_->estimate[2] = 0.0;

  keyframe_laser_data_->true_pose[0] = 0.0;
  keyframe_laser_data_->true_pose[1] = 0.0;
  keyframe_laser_data_->true_pose[2] = 0.0;

  input_.laser_ref = keyframe_laser_data_;
  input_.laser_sens = curr_laser_data;

  // convert offset to base frame
  tf2::Transform pred_base_offset = base_from_laser_ * pred_laser_offset * laser_from_base_;

  // predicted new pose of the base from the last measured pose
  tf2::Transform pred_base_in_fixed = prev_base_in_fixed_ * pred_base_offset;

  // predicted change of the laser's position from the keyframe in the laser frame
  tf2::Transform pr_ch_l = laser_from_base_ * (keyframe_base_in_fixed_.inverse() * pred_base_in_fixed) * base_from_laser_;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf2::getYaw(pr_ch_l.getRotation());

  // free covariance matrices, if they are allocated, to avoid leaking memory
  delete output_.cov_x_m;
  output_.cov_x_m = nullptr;
  delete output_.dx_dy1_m;
  output_.dx_dy1_m = nullptr;
  delete output_.dx_dy2_m;
  output_.dx_dy2_m = nullptr;

  // scan matching using Iterative Closest Point (ICP) from CSM
  sm_icp(&input_, &output_);
  tf2::Transform corr_ch;

  RCLCPP_DEBUG(get_logger(),"  keyframe: %lf, %lf", keyframe_base_in_fixed_.getOrigin().getX(), keyframe_base_in_fixed_.getOrigin().getY());
  RCLCPP_DEBUG(get_logger(),"  prev: %lf, %lf", prev_base_in_fixed_.getOrigin().getX(), prev_base_in_fixed_.getOrigin().getY());
  RCLCPP_DEBUG(get_logger(),"  pred prev laser offset: %lf, %lf", pred_laser_offset.getOrigin().getX(), pred_laser_offset.getOrigin().getY());
  RCLCPP_DEBUG(get_logger(),"  pred prev base offset: %lf, %lf", pred_base_offset.getOrigin().getX(), pred_base_offset.getOrigin().getY());
  RCLCPP_DEBUG(get_logger(),"  pred base: %lf, %lf", pred_base_in_fixed.getOrigin().getX(), pred_base_in_fixed.getOrigin().getY());
  RCLCPP_DEBUG(get_logger(),"  pred keyframe laser offset: %lf, %lf", pr_ch_l.getOrigin().getX(), pr_ch_l.getOrigin().getY());

  if (!output_.valid) {
    corr_ch.setIdentity();
    RCLCPP_WARN(get_logger(),"Error in scan matching. Creating new keyframe ...");

    // generate a new keyframe
    ld_free(keyframe_laser_data_);
    keyframe_laser_data_ = curr_laser_data;
    keyframe_base_in_fixed_ = prev_laser_in_tf_odom_ * base_from_laser_;
    base_in_fixed_ = keyframe_base_in_fixed_;
    prev_base_in_fixed_ = keyframe_base_in_fixed_;

    publishKeyframe(scan_msg);

    return false;
  }

  // the correction of the laser's position, in the laser frame
  tf2::Transform corr_ch_l;
  createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

  // the correction of the base's position, in the base frame
  corr_ch = base_from_laser_ * corr_ch_l * laser_from_base_;

  // update the pose in the world frame
  base_in_fixed_ = keyframe_base_in_fixed_ * corr_ch;

  RCLCPP_DEBUG(get_logger(),"  meas keyframe laser offset: %lf, %lf", corr_ch_l.getOrigin().getX(), corr_ch_l.getOrigin().getY());
  RCLCPP_DEBUG(get_logger(),"  meas keyframe base offset: %lf, %lf", corr_ch.getOrigin().getX(), corr_ch.getOrigin().getY());
  RCLCPP_DEBUG(get_logger(),"  meas base: %lf, %lf", base_in_fixed_.getOrigin().getX(), base_in_fixed_.getOrigin().getY());

  // icp covariance
  Eigen::Matrix2f icp_cov = Eigen::Matrix2f::Zero();
  float yaw_cov = 0.0;
  if (input_.do_compute_covariance) {
    icp_cov(0, 0) = (*output_.cov_x_m)(0, 0);
    icp_cov(0, 1) = (*output_.cov_x_m)(0, 1);
    icp_cov(1, 0) = (*output_.cov_x_m)(1, 0);
    icp_cov(1, 1) = (*output_.cov_x_m)(1, 1);

    // rotate into odom frame
    auto rotation = getLaserRotation(keyframe_base_in_fixed_);
    icp_cov = rotation * icp_cov * rotation.transpose();

    yaw_cov = (*output_.cov_x_m)(2, 2);
  }

  // degeneracy check covariance
  Eigen::Matrix2f degenerate_cov = Eigen::Matrix2f::Zero();
  if (degeneracy_check_) {
    auto degenerate_axis = checkAxisDegeneracy(*curr_laser_data, 0.1, scan_msg->header.frame_id, scan_msg->header.stamp);
    float degeneracy = degenerate_axis.norm();
    if (degeneracy == 0) {
      // error condition, set degenerate covariance for both axes
      degenerate_cov(0, 0) = degeneracy_cov_scale_ + degeneracy_cov_offset_;
      degenerate_cov(1, 1) = degeneracy_cov_scale_ + degeneracy_cov_offset_;;
    }
    else {
      // scale the degenerate axis
      Eigen::Vector2f degenerate_axis_scaled = degenerate_axis * std::pow(degeneracy, degeneracy_cov_ramp_) * degeneracy_cov_scale_;
      Eigen::Vector2f degenerate_axis_offset = degenerate_axis.normalized() * degeneracy_cov_offset_;
      degenerate_axis = degenerate_axis_scaled + degenerate_axis_offset;

      // create covariance matrix
      degenerate_cov = degenerate_axis * degenerate_axis.transpose();

      // rotate into odom frame
      auto rotation = getLaserRotation(base_in_fixed_);
      degenerate_cov = rotation * degenerate_cov * rotation.transpose();
    }
  }

  if (odom_pub_->get_subscription_count() > 0) {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = scan_msg->header.stamp;;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    tf2::toMsg(base_in_fixed_, odom_msg.pose.pose);

    // elapsed time between consecutive scans
    double dt = (rclcpp::Time(scan_msg->header.stamp) - prev_stamp_).nanoseconds() / 1e+9;

    // Get pose difference in base frame and calculate velocities
    auto pose_difference = prev_base_in_fixed_.inverse() * base_in_fixed_;
    odom_msg.twist.twist.linear.x = pose_difference.getOrigin().getX()/dt;
    odom_msg.twist.twist.linear.y = pose_difference.getOrigin().getY()/dt;
    odom_msg.twist.twist.angular.z = tf2::getYaw(pose_difference.getRotation())/dt;

    odom_msg.pose.covariance[0] = icp_cov(0, 0) * xy_cov_scale_ + xy_cov_offset_ + degenerate_cov(0, 0);
    odom_msg.pose.covariance[1] = icp_cov(0, 1) * xy_cov_scale_ + degenerate_cov(0, 1);
    odom_msg.pose.covariance[6] = icp_cov(1, 0) * xy_cov_scale_ + degenerate_cov(1, 0);
    odom_msg.pose.covariance[7] = icp_cov(1, 1) * xy_cov_scale_ + xy_cov_offset_ + degenerate_cov(1, 1);
    odom_msg.pose.covariance[35] = yaw_cov * heading_cov_scale_ + heading_cov_offset_;

    odom_msg.twist.covariance = odom_msg.pose.covariance;

    odom_pub_->publish(odom_msg);
  }

  if (pose_pub_->get_subscription_count() > 0) {
    // stamped Pose message
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = scan_msg->header.stamp;
    pose_msg.header.frame_id = odom_frame_;
    tf2::toMsg(base_in_fixed_, pose_msg.pose.pose);

    pose_msg.pose.covariance[0] = icp_cov(0, 0) * xy_cov_scale_ + xy_cov_offset_ + degenerate_cov(0, 0);
    pose_msg.pose.covariance[1] = icp_cov(0, 1) * xy_cov_scale_ + degenerate_cov(0, 1);
    pose_msg.pose.covariance[6] = icp_cov(1, 0) * xy_cov_scale_ + degenerate_cov(1, 0);
    pose_msg.pose.covariance[7] = icp_cov(1, 1) * xy_cov_scale_ + xy_cov_offset_ + degenerate_cov(1, 1);
    pose_msg.pose.covariance[35] = yaw_cov * heading_cov_scale_ + heading_cov_offset_;

    pose_pub_->publish(pose_msg);
  }

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.transform = tf2::toMsg(base_in_fixed_);

    tf_msg.header.stamp = scan_msg->header.stamp;;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_broadcaster_->sendTransform (tf_msg);
  }

  if (newKeyframeNeeded(corr_ch)) {
    RCLCPP_INFO(get_logger(),"Creating new keyframe ...");
    // generate a keyframe
    ld_free(keyframe_laser_data_);
    keyframe_laser_data_ = curr_laser_data;
    keyframe_base_in_fixed_ = base_in_fixed_;

    publishKeyframe(scan_msg);
  }
  else {
    ld_free(curr_laser_data);
  }

  prev_base_in_fixed_ = base_in_fixed_;
  return true;
}

bool LaserScanMatcher::newKeyframeNeeded(const tf2::Transform& d) {
  if (std::fabs(tf2::getYaw(d.getRotation())) > min_travel_heading_ * M_PI / 180.0) {
    return true;
  }

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x * x + y * y > min_travel_distance_ * min_travel_distance_) {
    return true;
  }

  return false;
}

void LaserScanMatcher::publishKeyframe(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  if (keyframe_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 laser_cloud;
    laser_projector_.projectLaser(*scan_msg, laser_cloud);

    // transform from laser to fixed frame
    auto transform = keyframe_base_in_fixed_ * base_from_laser_;
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.transform = tf2::toMsg(transform);
    sensor_msgs::msg::PointCloud2 fixed_cloud;
    tf2::doTransform (laser_cloud, fixed_cloud, transform_msg);
    fixed_cloud.header.frame_id = odom_frame_;
    fixed_cloud.header.stamp = scan_msg->header.stamp;
    keyframe_pub_->publish(fixed_cloud);
  }
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan, LDP& ldp) {
  unsigned int n = scan->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++) {
    // Calculate position in laser frame
    double r = scan->ranges[i];
    if ((r > scan->range_min) && (r < scan->range_max)) {
      // Fill in laser scan data
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }
    ldp->theta[i] = scan->angle_min + i * scan->angle_increment;
    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t) {
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

Eigen::Vector2f LaserScanMatcher::checkAxisDegeneracy(const laser_data& scan, float baseline, const std::string& laser_frame, rclcpp::Time stamp) {

  if (!scan.corr) {
    RCLCPP_WARN(get_logger(), "No scan correspondences!");
    return {0, 0};
  }

  // get all of the valid points
  std::vector<Eigen::Vector2f> points;
  std::vector<int> original_indices;
  points.reserve(scan.nrays);
  original_indices.reserve(scan.nrays);
  for (int i = 0; i < scan.nrays; i++) {
    if (scan.valid[i]) {
      points.emplace_back(scan.points[i].p[0], scan.points[i].p[1]);
      original_indices.push_back(i);
    }
  }

  if (points.empty()) {
    RCLCPP_WARN(get_logger(), "No valid points in scan!");
    return {0, 0};
  }

  float half_baseline = 0.5f * baseline;

  std::vector<Eigen::Vector2f> normals;
  normals.reserve(points.size() * 2);

  // compute sequential distance of each valid point in the scan from the first point
  std::vector<float> distances(points.size(), 0.0f);
  for (int i = 1; i < points.size(); i++) {
    distances[i] = distances[i - 1] + (points[i] - points[i - 1]).norm();
  }

  // calculate the normal vector of each valid point that has a valid correspondence
  int start_idx = 0;
  int end_idx = 0;
  for (int i = 0; i < points.size(); i++) {

    // check if correspondence is valid
    if (!scan.corr[original_indices[i]].valid) {
      continue;
    }

    double current_dist = distances[i];

    // find the interpolated start point to measure the normal of the scan point
    auto start_pt = points[start_idx];
    if (current_dist > half_baseline) {
      float target_dist = current_dist - half_baseline;
      while (distances[start_idx] < target_dist) {
        start_idx++;
      }

      float r = half_baseline - (current_dist - distances[start_idx]);
      float l = distances[start_idx] - distances[start_idx - 1];
      float w = r / l;
      start_pt = points[start_idx - 1] * w + (1.0f - w) * points[start_idx];
    }

    // find the interpolated end point to measure the normal of the scan point
    auto end_pt = points[end_idx];
    if (distances.back() - current_dist > half_baseline) {
      float target_dist = current_dist + half_baseline;
      while (distances[end_idx] < target_dist) {
        end_idx++;
      }

      float r = half_baseline - (distances[end_idx - 1] - current_dist);
      float l = distances[end_idx] - distances[end_idx - 1];
      float w = r / l;
      end_pt = points[end_idx] * w + (1.0f - w) * points[end_idx - 1];
    }

    Eigen::Vector2f v = (end_pt - start_pt).normalized();
    normals.emplace_back(-v[1], v[0]);
  }

  if (normals.empty()) {
    RCLCPP_WARN(get_logger(), "No valid correspondences in scan!");
    return {0, 0};
  }

  Eigen::Map<Eigen::Matrix<float,2,Eigen::Dynamic>> N(normals.data()->data(), 2, normals.size());

  // compute the SVD of the normal vector matrix. if sigma_min << sigma_max,
  // the pointcloud is degenerate (i.e. uninformative in a given direction)
  auto svd = N.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto singular_values = svd.singularValues();
  int min_idx = 0;
  int max_idx = 1;
  float min_val = singular_values(0);
  float max_val = singular_values(1);
  if (min_val > max_val) {
    std::swap(min_idx, max_idx);
    std::swap(min_val, max_val);
  }

  if (max_val == 0) {
    return {0, 0};
  }

  float degeneracy = std::max(0.0001, 1.0 - std::min(1.0f, min_val / max_val));
  Eigen::Vector2f primary_axis = svd.matrixU().col(max_idx);
  primary_axis.normalize();

  Eigen::Vector2f secondary_axis = { -primary_axis[1], primary_axis[0] };

  return secondary_axis * degeneracy;
}

Eigen::Matrix2f LaserScanMatcher::getLaserRotation(const tf2::Transform& odom_pose) {
  tf2::Transform laser_in_fixed = odom_pose * laser_from_base_;
  tf2::Matrix3x3 fixed_from_laser_rot(laser_in_fixed.getRotation());
  double r,p,y;
  fixed_from_laser_rot.getRPY(r, p, y);
  Eigen::Rotation2Df t(y);
  return t.toRotationMatrix();
}

}  // namespace scan_tools

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_tools::LaserScanMatcher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
