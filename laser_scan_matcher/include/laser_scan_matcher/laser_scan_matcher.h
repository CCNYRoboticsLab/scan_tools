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
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <laser_scan_matcher/param_handler.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <csm/csm.h>
#undef min
#undef max

namespace scan_tools {

class LaserScanMatcher: public rclcpp::Node
{
  public:
    LaserScanMatcher();
    ~LaserScanMatcher() = default;

  private:
    // **** ros

    rclcpp::TimerBase::SharedPtr init_timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_stamped_subscriber_;

    std::shared_ptr<tf2_ros::TransformListener> tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    tf2::Transform base_from_laser_; // static, cached
    tf2::Transform laser_from_base_; // static, cached

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

    // **** coordinate parameters
    std::string base_frame_ = "base_link";
    std::string odom_frame_ = "odom";
    bool publish_tf_ = false;
    double tf_timeout_ = 0.1;

    // **** keyframe parameters
    double min_travel_distance_ = 0.5;
    double min_travel_heading_ = 30.0;

    // **** covariance parameters
    double xy_cov_scale_ = 1.0;
    double xy_cov_offset_ = 0.0;
    double heading_cov_scale_ = 1.0;
    double heading_cov_offset_ = 0.0;

    // **** What predictions are available to speed up the ICP?
    // 1) imu - [theta] from imu yaw angle - /imu topic
    // 2) odom - [x, y, theta] from wheel odometry - /odom topic
    // 3) tf - [x, y, theta] from /tf topic
    // 3) velocity [vx, vy, vtheta], usually from ab-filter - /vel.
    // If more than one is enabled, priority is tf > imu > odom > velocity
    bool use_imu_ = true;
    bool use_odom_ = true;
    bool use_tf_ = false;
    bool use_vel_ = false;
    bool stamped_vel_ = false;

    // **** state variables
    bool initialized_ = false;
    ParamHandler params_;

    tf2::Transform last_base_in_fixed_;      // pose of the base of the last scan in fixed frame
    tf2::Transform keyframe_base_in_fixed_;  // pose of the base of last keyframe scan in fixed frame

    rclcpp::Time last_icp_time_;

    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_msg_;
    geometry_msgs::msg::Twist::SharedPtr latest_vel_msg_;

    tf2::Quaternion last_used_imu_orientation_;
    tf2::Transform last_used_odom_pose_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    sm_params input_;
    sm_result output_;
    LDP keyframe_laser_data_;

      // **** methods

    void initParams();
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan, LDP& ldp);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void velCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
    void velStmpCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);

    void createCache(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg);

    /**
     * Cache the static transform between the base and laser.
     *
     * @param[in] frame_id  The laser frame.
     *
     * @returns True if the transform was found, false otherwise.
     */
    bool getBaseToLaserTf (const std::string& frame_id);

    bool getTfOffset(
      const std::string& frame_id,
      const rclcpp::Time& target_stamp,
      const rclcpp::Time& source_stamp,
      tf2::Transform& transform);

    bool newKeyframeNeeded(const tf2::Transform& d);

    tf2::Transform getPrediction(const rclcpp::Time& stamp);

    void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);

    Eigen::Matrix2f getLaserRotation(const tf2::Transform& odom_pose);
};

}  // namespace scan_tools
