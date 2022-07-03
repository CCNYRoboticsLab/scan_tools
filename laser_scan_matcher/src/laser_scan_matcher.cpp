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

#include <ros2_laser_scan_matcher/laser_scan_matcher.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#undef min
#undef max

namespace scan_tools
{

void LaserScanMatcher::add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description, const std::string & additional_constraints,
    bool read_only)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }


LaserScanMatcher::LaserScanMatcher() : Node("laser_scan_matcher"), initialized_(false)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // Initiate parameters

   RCLCPP_INFO(get_logger(), "Creating laser_scan_matcher");
  add_parameter("publish_tf",   rclcpp::ParameterValue(false),
    "Whether to publish tf transform from 'odom_frame' to 'base_frame'");

  add_parameter("xy_cov_scale", rclcpp::ParameterValue(xy_cov_scale_),"");
  add_parameter("xy_cov_offset", rclcpp::ParameterValue(xy_cov_offset_),"");
  add_parameter("heading_cov_scale", rclcpp::ParameterValue(heading_cov_scale_),"");
  add_parameter("heading_cov_offset", rclcpp::ParameterValue(heading_cov_offset_),"");

  add_parameter("base_frame", rclcpp::ParameterValue(std::string("base_link")),
    "Which frame to use for the robot base");
  add_parameter("odom_frame", rclcpp::ParameterValue(std::string("odom")),
    "Which frame to use for the odom");
  add_parameter("kf_dist_linear", rclcpp::ParameterValue(0.10),
    "When to generate keyframe scan.");
  add_parameter("kf_dist_angular", rclcpp::ParameterValue(10.0* (M_PI/180.0)),
    "When to generate keyframe scan.");



  // CSM parameters - comments copied from algos.h (by Andrea Censi)
  add_parameter("max_angular_correction_deg", rclcpp::ParameterValue(45.0),
    "Maximum angular displacement between scansr.");

  add_parameter("max_linear_correction", rclcpp::ParameterValue(0.5),
    "Maximum translation between scans (m).");

  add_parameter("max_iterations", rclcpp::ParameterValue(10),
    "Maximum ICP cycle iterationsr.");

  add_parameter("epsilon_xy", rclcpp::ParameterValue(0.000001),
   "A threshold for stopping (m).");

  add_parameter("epsilon_theta", rclcpp::ParameterValue(0.000001),
    "A threshold for stopping (rad).");

  add_parameter("max_correspondence_dist", rclcpp::ParameterValue(0.3),
    "Maximum distance for a correspondence to be valid.");

  add_parameter("sigma", rclcpp::ParameterValue(0.010),
    "Noise in the scan (m).");

  add_parameter("use_corr_tricks", rclcpp::ParameterValue(1),
    "Use smart tricks for finding correspondences.");

  add_parameter("restart", rclcpp::ParameterValue(0),
    "Restart if error is over threshold.");

  add_parameter("restart_threshold_mean_error", rclcpp::ParameterValue(0.01),
    "Threshold for restarting.");

  add_parameter("restart_dt", rclcpp::ParameterValue(1.0),
   "Displacement for restarting. (m).");

  add_parameter("restart_dtheta", rclcpp::ParameterValue(0.1),
    "Displacement for restarting. (rad).");

  add_parameter("clustering_threshold", rclcpp::ParameterValue(0.25),
    "Max distance for staying in the same clustering.");

  add_parameter("orientation_neighbourhood", rclcpp::ParameterValue(20),
    "Number of neighbour rays used to estimate the orientation.");

  add_parameter("use_point_to_line_distance", rclcpp::ParameterValue(1),
    "If 0, it's vanilla ICP.");

  add_parameter("do_alpha_test", rclcpp::ParameterValue(0),
   " Discard correspondences based on the angles.");

  add_parameter("do_alpha_test_thresholdDeg", rclcpp::ParameterValue(20.0),
    "Discard correspondences based on the angles - threshold angle, in degrees.");

  add_parameter("outliers_maxPerc", rclcpp::ParameterValue(0.9),
    "Percentage of correspondences to consider: if 0.9, \
        always discard the top 10% of correspondences with more error");


  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  add_parameter("outliers_adaptive_order", rclcpp::ParameterValue(0.7),
    "");

  add_parameter("outliers_adaptive_mult", rclcpp::ParameterValue(2.0),
    "");

  // If you already have a guess of the solution, you can compute the polar
  // angle
  // of the points of one scan in the new position. If the polar angle is not a
  // monotone
  // function of the readings index, it means that the surface is not visible in
  // the
  // next position. If it is not visible, then we don't use it for matching.
  add_parameter("do_visibility_test", rclcpp::ParameterValue(0),
    "");

  add_parameter("outliers_remove_doubles", rclcpp::ParameterValue(1),
    "No two points in laser_sens can have the same corr.");

  add_parameter("do_compute_covariance", rclcpp::ParameterValue(true),
    "If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov");

  add_parameter("debug_verify_tricks", rclcpp::ParameterValue(0),
    " Checks that find_correspondences_tricks gives the right answer.");

  add_parameter("use_ml_weights", rclcpp::ParameterValue(0),
    "If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to \
         compute the incidence beta, and the factor (1/cos^2(beta)) used to weight the \
         correspondence.");

  add_parameter("use_sigma_weights", rclcpp::ParameterValue(0),
    " If 1, the field 'readings_sigma' in the second scan is used to weight the correspondence by 1/sigma^2");

  base_frame_ = get_parameter("base_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  kf_dist_linear_  = get_parameter("kf_dist_linear").as_double();
  kf_dist_angular_ = get_parameter("kf_dist_angular").as_double();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  xy_cov_scale_ = get_parameter("xy_cov_scale").as_double();
  xy_cov_offset_ = get_parameter("xy_cov_offset").as_double();
  heading_cov_scale_ = get_parameter("heading_cov_scale").as_double();
  heading_cov_offset_ = get_parameter("heading_cov_offset").as_double();

  kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

  input_.max_angular_correction_deg = get_parameter("max_angular_correction_deg").as_double();
  input_.max_linear_correction = get_parameter("max_linear_correction").as_double();
  input_.max_iterations = get_parameter("max_iterations").as_int();
  input_.epsilon_xy = get_parameter("epsilon_xy").as_double();
  input_.epsilon_theta = get_parameter("epsilon_theta").as_double();
  input_.max_correspondence_dist = get_parameter("max_correspondence_dist").as_double();
  input_.sigma = get_parameter("sigma").as_double();
  input_.use_corr_tricks = get_parameter("use_corr_tricks").as_int();
  input_.restart = get_parameter("restart").as_int();
  input_.restart_threshold_mean_error = get_parameter("restart_threshold_mean_error").as_double();
  input_.restart_dt = get_parameter("restart_dt").as_double();
  input_.restart_dtheta = get_parameter("restart_dtheta").as_double();
  input_.clustering_threshold = get_parameter("clustering_threshold").as_double();
  input_.orientation_neighbourhood = get_parameter("orientation_neighbourhood").as_int();
  input_.use_point_to_line_distance = get_parameter("use_point_to_line_distance").as_int();
  input_.do_alpha_test = get_parameter("do_alpha_test").as_int();
  input_.do_alpha_test_thresholdDeg = get_parameter("do_alpha_test_thresholdDeg").as_double();
  input_.outliers_maxPerc = get_parameter("outliers_maxPerc").as_double();
  input_.outliers_adaptive_order = get_parameter("outliers_adaptive_order").as_double();
  input_.outliers_adaptive_mult = get_parameter("outliers_adaptive_mult").as_double();
  input_.do_visibility_test = get_parameter("do_visibility_test").as_int();
  input_.outliers_remove_doubles = get_parameter("outliers_remove_doubles").as_int();
  input_.do_compute_covariance = get_parameter("do_compute_covariance").as_bool();
  input_.debug_verify_tricks = get_parameter("debug_verify_tricks").as_int();
  input_.use_ml_weights = get_parameter("use_ml_weights").as_int();
  input_.use_sigma_weights = get_parameter("use_sigma_weights").as_int();

  RCLCPP_WARN(get_logger(),"do_compute_covariance: %d", input_.do_compute_covariance);

  // State variables

  base_in_fixed_.setIdentity();
  prev_base_in_fixed_.setIdentity();
  keyframe_base_in_fixed_.setIdentity();
  prev_laser_in_tf_odom_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  // Subscribers
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanMatcher::scanCallback, this, std::placeholders::_1));
  tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  if (publish_tf_) {
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("laser_odom", rclcpp::SystemDefaultsQoS());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("laser_pose", rclcpp::SystemDefaultsQoS());
  keyframe_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", rclcpp::SystemDefaultsQoS());
}

LaserScanMatcher::~LaserScanMatcher()
{

}

void LaserScanMatcher::createCache (const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}


void LaserScanMatcher::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  RCLCPP_INFO(get_logger(),"Scan callback:");
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
      RCLCPP_WARN(get_logger(),"  skipping scan");
      return;
    }

    laserScanToLDP(scan_msg, keyframe_laser_data_);
    prev_stamp_ = scan_msg->header.stamp;
    prev_laser_in_tf_odom_ = laser_in_tf_odom;
    initialized_ = true;
  }

  RCLCPP_INFO(get_logger(),"  laser in tf odom: %lf, %lf", laser_in_tf_odom.getOrigin().getX(), laser_in_tf_odom.getOrigin().getY());
  RCLCPP_INFO(get_logger(),"  prev laser in tf odom: %lf, %lf", prev_laser_in_tf_odom_.getOrigin().getX(), prev_laser_in_tf_odom_.getOrigin().getY());

  auto pred_laser_offset = prev_laser_in_tf_odom_.inverse() * laser_in_tf_odom;

  if (processScan(scan_msg, pred_laser_offset)) {
    prev_laser_in_tf_odom_ = laser_in_tf_odom;
  }
  else {
    // TODO(malban): need to reset at some point?
    RCLCPP_WARN(get_logger(), "  failed to process scan");
  }
}

bool LaserScanMatcher::getBaseToLaserTf(const std::string& frame_id)
{
  try {
      auto msg = tf_buffer_->lookupTransform(base_frame_, frame_id, rclcpp::Time(0), rclcpp::Duration(10,0));
      tf2::fromMsg(msg.transform, base_from_laser_);
      laser_from_base_ = base_from_laser_.inverse();
  }
  catch (tf2::TransformException ex) {
    RCLCPP_INFO(get_logger(),"Could not get initial transform of base to laser frame, %s", ex.what());
    return false;
  }

  return true;
}

bool LaserScanMatcher::getLaserInTfOdom(const std::string& frame_id, const rclcpp::Time& stamp, tf2::Transform& transform)
{
  try {
    //                                                                          50 milliseconds
    auto msg = tf_buffer_->lookupTransform(odom_frame_, frame_id, stamp, rclcpp::Duration(0, 50000000));
    tf2::fromMsg(msg.transform, transform);
  }
  catch (tf2::TransformException ex)
  {
    RCLCPP_WARN(get_logger(),"Could not get transform laser to fixed frame, %s", ex.what());
    return false;
  }

  return true;
}

bool LaserScanMatcher::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, const tf2::Transform& pred_laser_offset)
{
  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (keyframe_laser_data_) has a pose of [0, 0, 0]
  // The new scan (curr_laser_data) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

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

  // elapsed time between consecutive scans
  double dt = (rclcpp::Time(scan_msg->header.stamp) - prev_stamp_).nanoseconds() / 1e+9;

  // predicted new pose of the base from prev_pose
  tf2::Transform pred_base_in_fixed =  prev_base_in_fixed_ * base_from_laser_ * pred_laser_offset;

  // predicted change of the laser's position from the keyframe in the laser frame
  tf2::Transform pr_ch_l = laser_from_base_ * (keyframe_base_in_fixed_.inverse()) * pred_base_in_fixed;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf2::getYaw(pr_ch_l.getRotation());

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // Scan matching - using point to line icp from CSM

  sm_icp(&input_, &output_);
  tf2::Transform corr_ch;

  RCLCPP_INFO(get_logger(),"  pred prev laser offset: %lf, %lf", pred_laser_offset.getOrigin().getX(), pred_laser_offset.getOrigin().getY());
  RCLCPP_INFO(get_logger(),"  pred base: %lf, %lf", pred_base_in_fixed.getOrigin().getX(), pred_base_in_fixed.getOrigin().getY());
  RCLCPP_INFO(get_logger(),"  pred keyframe laser offset: %lf, %lf", pr_ch_l.getOrigin().getX(), pr_ch_l.getOrigin().getY());

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame
    tf2::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    corr_ch = base_from_laser_ * corr_ch_l * laser_from_base_;

    // update the pose in the world frame
    base_in_fixed_ = keyframe_base_in_fixed_ * corr_ch;

    RCLCPP_INFO(get_logger(),"  meas keyframe laser offset: %lf, %lf", corr_ch.getOrigin().getX(), corr_ch.getOrigin().getY());
    RCLCPP_INFO(get_logger(),"  meas base: %lf, %lf", base_in_fixed_.getOrigin().getX(), base_in_fixed_.getOrigin().getY());
  }
  else
  {
    corr_ch.setIdentity();
    RCLCPP_WARN(get_logger(),"Error in scan matching");

    // TODO create new keyframe and try again

    return false;
  }

  if (odom_pub_->get_subscription_count() > 0)
  {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = scan_msg->header.stamp;;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    tf2::toMsg(base_in_fixed_, odom_msg.pose.pose);

    // Get pose difference in base frame and calculate velocities
    auto pose_difference = prev_base_in_fixed_.inverse() * base_in_fixed_;
    odom_msg.twist.twist.linear.x = pose_difference.getOrigin().getX()/dt;
    odom_msg.twist.twist.linear.y = pose_difference.getOrigin().getY()/dt;
    odom_msg.twist.twist.angular.z = tf2::getYaw(pose_difference.getRotation())/dt;

    if (input_.do_compute_covariance)
    {
      odom_msg.pose.covariance[0] = gsl_matrix_get(output_.cov_x_m, 0, 0);
      odom_msg.pose.covariance[1] = gsl_matrix_get(output_.cov_x_m, 0, 1);
      odom_msg.pose.covariance[6] = gsl_matrix_get(output_.cov_x_m, 1, 0);
      odom_msg.pose.covariance[7] = gsl_matrix_get(output_.cov_x_m, 1, 1);
      odom_msg.pose.covariance[35] = gsl_matrix_get(output_.cov_x_m, 2, 2);
    }

    odom_msg.pose.covariance[0] *= xy_cov_scale_;
    odom_msg.pose.covariance[0] += xy_cov_offset_;
    odom_msg.pose.covariance[1] *= xy_cov_scale_;
    odom_msg.pose.covariance[6] *= xy_cov_scale_;
    odom_msg.pose.covariance[7] *= xy_cov_scale_;
    odom_msg.pose.covariance[7] += xy_cov_offset_;
    odom_msg.pose.covariance[35] *= heading_cov_scale_;
    odom_msg.pose.covariance[35] += heading_cov_offset_;

    odom_msg.twist.covariance = odom_msg.pose.covariance;

    odom_pub_->publish(odom_msg);
  }

  if (pose_pub_->get_subscription_count() > 0) {
    // stamped Pose message
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = scan_msg->header.stamp;
    pose_msg.header.frame_id = odom_frame_;
    tf2::toMsg(base_in_fixed_, pose_msg.pose.pose);

    if (input_.do_compute_covariance)
    {
      pose_msg.pose.covariance[0] = gsl_matrix_get(output_.cov_x_m, 0, 0);
      pose_msg.pose.covariance[1] = gsl_matrix_get(output_.cov_x_m, 0, 1);
      pose_msg.pose.covariance[6] = gsl_matrix_get(output_.cov_x_m, 1, 0);
      pose_msg.pose.covariance[7] = gsl_matrix_get(output_.cov_x_m, 1, 1);
      pose_msg.pose.covariance[35] = gsl_matrix_get(output_.cov_x_m, 2, 2);
    }

    pose_msg.pose.covariance[0] *= xy_cov_scale_;
    pose_msg.pose.covariance[0] += xy_cov_offset_;
    pose_msg.pose.covariance[1] *= xy_cov_scale_;
    pose_msg.pose.covariance[6] *= xy_cov_scale_;
    pose_msg.pose.covariance[7] *= xy_cov_scale_;
    pose_msg.pose.covariance[7] += xy_cov_offset_;
    pose_msg.pose.covariance[35] *= heading_cov_scale_;
    pose_msg.pose.covariance[35] += heading_cov_offset_;

    pose_pub_->publish(pose_msg);
  }

  if (publish_tf_)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.transform = tf2::toMsg(base_in_fixed_);

    tf_msg.header.stamp = scan_msg->header.stamp;;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tfB_->sendTransform (tf_msg);
  }

  // **** swap old and new
  if (newKeyframeNeeded(corr_ch))
  {
    RCLCPP_INFO(get_logger(),"Creating new keyframe ...");
    // generate a keyframe
    ld_free(keyframe_laser_data_);
    keyframe_laser_data_ = curr_laser_data;
    keyframe_base_in_fixed_ = base_in_fixed_;

    if (keyframe_pub_->get_subscription_count() > 0) {
      // TODO publish keyframe in odom frame

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
  else
  {
    ld_free(curr_laser_data);

  }

  prev_base_in_fixed_ = base_in_fixed_;
  prev_stamp_ = scan_msg->header.stamp;
  return true;
}

bool LaserScanMatcher::newKeyframeNeeded(const tf2::Transform& d)
{
  if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_)
    return true;

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x * x + y * y > kf_dist_linear_sq_)
    return true;

  return false;
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan, LDP& ldp)
{
  unsigned int n = scan->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // Calculate position in laser frame
    double r = scan->ranges[i];
    if ((r > scan->range_min) && (r < scan->range_max))
    {
      // Fill in laser scan data
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
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


void LaserScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t)
{
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

}  // namespace scan_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_tools::LaserScanMatcher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
