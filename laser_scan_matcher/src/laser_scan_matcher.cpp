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

#include <chrono>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std::chrono_literals;

namespace scan_tools {

LaserScanMatcher::LaserScanMatcher() :
  rclcpp::Node("laser_scan_matcher"),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock()))
{
  RCLCPP_INFO(get_logger(), "Starting LaserScanMatcher");

  init_timer_ = create_wall_timer(100ms, [&]()
  {
    // **** init parameters
    initParams();

    // **** state variables
    last_base_in_fixed_.setIdentity();
    keyframe_base_in_fixed_.setIdentity();
    last_used_odom_pose_.setIdentity();
    input_.laser[0] = 0.0;
    input_.laser[1] = 0.0;
    input_.laser[2] = 0.0;

    // Initialize output_ vectors as Null for error-checking
    output_.cov_x_m = nullptr;
    output_.dx_dy1_m = nullptr;
    output_.dx_dy2_m = nullptr;

    // **** publishers
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("laser_pose", rclcpp::SystemDefaultsQoS());

    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    // **** subscribers
    scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanMatcher::scanCallback, this, std::placeholders::_1));
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (use_imu_)
    {
      imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(), std::bind(&LaserScanMatcher::imuCallback, this, std::placeholders::_1));
    }
    if (use_odom_)
    {
      odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&LaserScanMatcher::odomCallback, this, std::placeholders::_1));
    }
    if (use_vel_)
    {
      if (stamped_vel_)
      {
        vel_stamped_subscriber_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "vel", rclcpp::SensorDataQoS(), std::bind(&LaserScanMatcher::velStmpCallback, this, std::placeholders::_1));
      }
      else
      {
        vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "vel", rclcpp::SensorDataQoS(), std::bind(&LaserScanMatcher::velCallback, this, std::placeholders::_1));
      }
    }
  });
}

void LaserScanMatcher::initParams() {
  init_timer_->cancel();
  params_.initialize(shared_from_this());

  // static parameters
  base_frame_ = params_.param("base_frame", std::string("base_link"), "Which frame to use for the robot base");
  odom_frame_ = params_.param("odom_frame", std::string("odom"), "Which frame to track odometry in");
  publish_tf_ = params_.param("publish_tf", true, "Whether to publish tf transform from 'odom_frame' to 'base_frame'");
  use_imu_ = params_.param("use_imu", true, "Whether to use imu messages for predicting the sensor yaw");
  use_odom_ = params_.param("use_odom", true, "Whether to use odom messages for predicting the sensor pose");
  use_vel_ = params_.param("use_vel", false, "Whether to use twist messages for predicting the sensor pose");
  stamped_vel_ = params_.param("stamped_vel", false, "Whether to subscribe to stamped twist messages.");

  // dynamic parameters
  params_.register_param(&tf_timeout_, "tf_timeout", 0.1, "TF timeout in seconds.", 0.0, 10.0);
  params_.register_param(&xy_cov_scale_, "xy_cov_scale", 1.0, "Scaling to apply to xy position covariance", 0.0, 1e8);
  params_.register_param(&xy_cov_offset_, "xy_cov_offset", 0.0, "Offset to apply to xy position covariance", 0.0, 10.0);
  params_.register_param(&heading_cov_scale_, "heading_cov_scale", 1.0, "Scaling to apply to heading covariance", 0.0, 1e8);
  params_.register_param(&heading_cov_offset_, "heading_cov_offset", 0.0, "Offset to apply to heading covariance", 0.0, 10.0);
  params_.register_param(&min_travel_distance_, "min_travel_distance", 0.5, "Distance in meters to trigger a new keyframe", 0.0, 10.0);
  params_.register_param(&min_travel_heading_, "min_travel_heading", 30.0, "Angle in degrees to trigger a new keyframe.", 0.0, 180.0);
  params_.register_param(&use_tf_, "use_tf", false, "Whether to use tf for predicting the sensor pose");

  // CSM parameters - comments copied from algos.h (by Andrea Censi)
  params_.register_param(&input_.max_angular_correction_deg, "max_angular_correction_deg", 45.0, "Maximum angular displacement between scans.", 0.0, 90.0);
  params_.register_param(&input_.max_linear_correction, "max_linear_correction", 0.5, "Maximum translation between scans (m).", 0.0, 10.0);
  params_.register_param(&input_.max_iterations, "max_iterations", 10, "Maximum ICP cycle iterations", 1, 100);
  params_.register_param(&input_.epsilon_xy, "epsilon_xy", 1e-6, "A threshold for stopping (m).", 1e-10, 1e-1);
  params_.register_param(&input_.epsilon_theta, "epsilon_theta", 1e-6, "A threshold for stopping (rad).", 1e-10, 1e-1);
  params_.register_param(&input_.max_correspondence_dist, "max_correspondence_dist", 0.3, "Maximum distance for a correspondence to be valid.", 0.0, 10.0);
  params_.register_param(&input_.sigma, "sigma", 0.01, "Noise in the scan (m).", 0.0, 1.0);
  params_.register_param(&input_.use_corr_tricks, "use_corr_tricks", true, "Use smart tricks for finding correspondences.");
  params_.register_param(&input_.restart, "restart", false, "Restart if error is over threshold.");
  params_.register_param(&input_.restart_threshold_mean_error, "restart_threshold_mean_error", 0.01, "Threshold for restarting.", 0.0, 1.0);
  params_.register_param(&input_.restart_dt, "restart_dt", 1.0, "Displacement for restarting. (m).", 0.0, 10.0);
  params_.register_param(&input_.restart_dtheta, "restart_dtheta", 0.1, "Displacement for restarting. (rad).", 0.0, M_PI_2);
  params_.register_param(&input_.clustering_threshold, "clustering_threshold", 0.25, "Max distance for staying in the same clustering.", 0.0, 1.0);
  params_.register_param(&input_.orientation_neighbourhood, "orientation_neighbourhood", 20, "Number of neighbour rays used to estimate the orientation.", 0, 100);
  params_.register_param(&input_.use_point_to_line_distance, "use_point_to_line_distance", true, "If false, it's vanilla ICP.");
  params_.register_param(&input_.do_alpha_test, "do_alpha_test", false, "Discard correspondences based on the angles.");
  params_.register_param(&input_.do_alpha_test_thresholdDeg, "alpha_test_threshold", 20.0, "Threshold angle to discard correspondences, in degrees.", 0.0, 90.0);
  params_.register_param(&input_.outliers_maxPerc, "outliers_max_perc", 0.9, "Percentage [0, 1] of correspondences to consider with lowest error", 0.0, 1.0);
  params_.register_param(&input_.outliers_adaptive_order, "outliers_adaptive_order", 0.7, "Percentile [0, 1] to use for adaptive outlier detection", 0.0, 1.0);
  params_.register_param(&input_.outliers_adaptive_mult, "outliers_adaptive_mult", 2.0, "Threshold multiplier for error at chosen percentile", 0.0, 10.0);
  params_.register_param(&input_.do_visibility_test, "do_visibility_test", false, "Use visibility test trick for matching.");
  params_.register_param(&input_.outliers_remove_doubles, "outliers_remove_doubles", true, "Do not allow two different correspondences to share a point");
  params_.register_param(&input_.do_compute_covariance, "do_compute_covariance", true, "Compute the covariance of the ICP");
  params_.register_param(&input_.debug_verify_tricks, "debug_verify_tricks", false, "Checks that find_correspondences_tricks gives the right answer.");
  params_.register_param(&input_.use_ml_weights, "use_ml_weights", false, "Use the 'true_alpha' or 'alpha' field to weight the impact of each correspondence");
  params_.register_param(&input_.use_sigma_weights, "use_sigma_weights", false, "Use the 'readings_sigma' field of the second scan to weight the correspondence.");

  params_.registerCallback();
}

void LaserScanMatcher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  if (!latest_imu_msg_) {
    tf2::fromMsg(imu_msg->orientation, last_used_imu_orientation_);
  }
  latest_imu_msg_ = imu_msg;
}

void LaserScanMatcher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  if (!latest_odom_msg_) {
    tf2::fromMsg(odom_msg->pose.pose, last_used_odom_pose_);
  }
  latest_odom_msg_ = odom_msg;
}

void LaserScanMatcher::velCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  latest_vel_msg_ = twist_msg;
}

void LaserScanMatcher::velStmpCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  latest_vel_msg_ = std::make_shared<geometry_msgs::msg::Twist>(twist_msg->twist);
}

void LaserScanMatcher::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  // **** if first scan, cache the tf from base to the scanner

  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      return;
    }

    laserScanToLDP(scan_msg, keyframe_laser_data_);
    last_icp_time_ = scan_msg->header.stamp;
    initialized_ = true;
  }

  processScan(scan_msg);
}

void LaserScanMatcher::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
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

  tf2::Transform last_base = last_base_in_fixed_;

  // **** estimated change since last scan

  // get the predicted offset of the scan base pose from the last scan base pose
  tf2::Transform pred_last_base_offset = getPrediction(scan_msg->header.stamp);

  // calculate the predicted scan base pose by applying the predicted offset to the last scan base pose
  tf2::Transform pred_base_in_fixed = last_base_in_fixed_ * pred_last_base_offset;

  // calculate the offset between the keyframe base pose and predicted scan base pose
  tf2::Transform pred_keyframe_base_offset = keyframe_base_in_fixed_.inverse() * pred_base_in_fixed;

  // convert the predicted offset from the keyframe base frame to be in the keyframe laser frame
  tf2::Transform pred_keyframe_laser_offset = laser_from_base_ * pred_keyframe_base_offset * base_from_laser_ ;

  input_.first_guess[0] = pred_keyframe_laser_offset.getOrigin().getX();
  input_.first_guess[1] = pred_keyframe_laser_offset.getOrigin().getY();
  input_.first_guess[2] = tf2::getYaw(pred_keyframe_laser_offset.getRotation());

  // free covariance matrices, if they are allocated, to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = nullptr;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = nullptr;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = nullptr;
  }

  // *** scan match - using point to line icp from CSM

  sm_icp(&input_, &output_);
  tf2::Transform meas_keyframe_base_offset;

  if (output_.valid)
  {
    // the measured offset of the scan from the keyframe in the keyframe laser frame
    tf2::Transform meas_keyframe_laser_offset;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], meas_keyframe_laser_offset);

    // convert the measured offset from the keyframe laser frame to the keyframe base frame
    meas_keyframe_base_offset = base_from_laser_ * meas_keyframe_laser_offset * laser_from_base_;

    // calculate the measured pose of the scan in the fixed frame
    last_base_in_fixed_ = keyframe_base_in_fixed_ * meas_keyframe_base_offset;

    // **** publish

    Eigen::Matrix2f xy_cov = Eigen::Matrix2f::Zero();
    float yaw_cov = 0.0;
    if (input_.do_compute_covariance)
    {
      // get covariance from ICP
      xy_cov(0, 0) = gsl_matrix_get(output_.cov_x_m, 0, 0);
      xy_cov(0, 1) = gsl_matrix_get(output_.cov_x_m, 0, 1);
      xy_cov(1, 0) = gsl_matrix_get(output_.cov_x_m, 1, 0);
      xy_cov(1, 1) = gsl_matrix_get(output_.cov_x_m, 1, 1);

      // rotate xy covariance from the keyframe into odom frame
      auto rotation = getLaserRotation(keyframe_base_in_fixed_);
      xy_cov = rotation * xy_cov * rotation.transpose();

      yaw_cov = gsl_matrix_get(output_.cov_x_m, 2, 2);
    }

    if (pose_publisher_->get_subscription_count() > 0) {
      // stamped Pose message
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

      pose_msg.header.stamp = scan_msg->header.stamp;
      pose_msg.header.frame_id = odom_frame_;
      tf2::toMsg(last_base_in_fixed_, pose_msg.pose.pose);

      pose_msg.pose.covariance[0] = xy_cov(0, 0) * xy_cov_scale_ + xy_cov_offset_;
      pose_msg.pose.covariance[1] = xy_cov(0, 1) * xy_cov_scale_;
      pose_msg.pose.covariance[6] = xy_cov(1, 0) * xy_cov_scale_;
      pose_msg.pose.covariance[7] = xy_cov(1, 1) * xy_cov_scale_ + xy_cov_offset_;
      pose_msg.pose.covariance[35] = yaw_cov * heading_cov_scale_ + heading_cov_offset_;

      pose_publisher_->publish(pose_msg);
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.transform = tf2::toMsg(last_base_in_fixed_);

      tf_msg.header.stamp = scan_msg->header.stamp;;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_broadcaster_->sendTransform (tf_msg);
    }
  }
  else {
    meas_keyframe_base_offset.setIdentity();
    RCLCPP_WARN(get_logger(),"Error in scan matching. Creating new keyframe ...");

    // generate a new keyframe
    ld_free(keyframe_laser_data_);
    keyframe_laser_data_ = curr_laser_data;
    keyframe_base_in_fixed_ = last_base;
    last_base_in_fixed_ = keyframe_base_in_fixed_;

    last_icp_time_ = scan_msg->header.stamp;
    return;
  }

  if (newKeyframeNeeded(meas_keyframe_base_offset))
  {
    // generate a keyframe
    RCLCPP_DEBUG(get_logger(), "Creating new keyframe ...");
    ld_free(keyframe_laser_data_);
    keyframe_laser_data_ = curr_laser_data;
    keyframe_base_in_fixed_ = last_base_in_fixed_;
  }
  else
  {
    ld_free(curr_laser_data);
  }

  last_icp_time_ = scan_msg->header.stamp;

  return;
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

bool LaserScanMatcher::getTfOffset(
  const std::string& frame_id,
  const rclcpp::Time& target_stamp,
  const rclcpp::Time& source_stamp,
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
    auto msg = tf_buffer_->lookupTransform(
      frame_id, target_stamp, frame_id, source_stamp, odom_frame_, rclcpp::Duration::from_seconds(tf_timeout_));
    tf2::fromMsg(msg.transform, transform);
  }
  catch (tf2::TransformException ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Could not get transform: %s", ex.what());
    return false;
  }

  return true;
}

tf2::Transform LaserScanMatcher::getPrediction(const rclcpp::Time& stamp) {
  // **** base case - no input available, use zero-motion model
  tf2::Transform pred_last_base_offset = tf2::Transform::getIdentity();

  // **** use velocity (for example from ab-filter)
  if (use_vel_)
  {
    double dt = (stamp - last_icp_time_).seconds();
    // NOTE: this assumes the velocity is in the base frame and that the base
    //       and laser frames share the same x,y and z axes
    double pr_ch_x = dt * latest_vel_msg_->linear.x;
    double pr_ch_y = dt * latest_vel_msg_->linear.y;
    double pr_ch_a = dt * latest_vel_msg_->angular.z;

    createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pred_last_base_offset);
  }

  // **** use wheel odometry
  if (use_odom_ && latest_odom_msg_)
  {
    // NOTE: this assumes the odometry is in the base frame
    tf2::Transform latest_odom_pose;
    tf2::fromMsg(latest_odom_msg_->pose.pose, latest_odom_pose);

    pred_last_base_offset = last_used_odom_pose_.inverse() * latest_odom_pose;

    last_used_odom_pose_ = latest_odom_pose;
  }

  // **** use imu
  if (use_imu_ && latest_imu_msg_)
  {
    // NOTE: this assumes the imu is in the base frame
    tf2::Quaternion latest_imu_orientation;
    tf2::fromMsg(latest_imu_msg_->orientation, latest_imu_orientation);

    tf2::Quaternion imu_orientation_offset = last_used_imu_orientation_.inverse() * latest_imu_orientation;
    pred_last_base_offset.setRotation(imu_orientation_offset);

    last_used_imu_orientation_ = latest_imu_orientation;
  }

  // **** use tf
  if (use_tf_)
  {
    tf2::Transform pred_last_base_offset_tf;
    if (getTfOffset(base_frame_, last_icp_time_, stamp, pred_last_base_offset_tf))
    {
      pred_last_base_offset = pred_last_base_offset_tf;
    }
  }

  return pred_last_base_offset;
}

void LaserScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t) {
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
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
