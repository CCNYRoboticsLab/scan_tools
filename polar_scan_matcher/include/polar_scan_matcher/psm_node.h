/*
*  Polar Scan Matcher
*  Copyright (C) 2010, CCNY Robotics Lab
*  Ivan Dryanovski <ivan.dryanovski@gmail.com>
*  William Morris <morris@ee.ccny.cuny.edu>
*  http://robotics.ccny.cuny.edu
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Polar Scan Matcher [1] algorithm written by A. Diosi
*
*  [1] A. Diosi and L. Kleeman, "Laser Scan Matching in Polar Coordinates with 
*  Application to SLAM " Proceedings of 2005 IEEE/RSJ International Conference 
*  on Intelligent Robots and Systems, August, 2005, Edmonton, Canada
*/

#ifndef POLAR_SCAN_MATCHER_PSM_NODE_H
#define POLAR_SCAN_MATCHER_PSM_NODE_H

#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "polar_scan_matcher/polar_match.h"

const std::string scanTopic_ = "scan";
const std::string poseTopic_ = "pose2D";

const double ROS_TO_PM = 100.0;   // convert from cm to m

class PSMNode
{
  private:

    PolarMatcher matcher_;

    ros::Subscriber scanSubscriber_;
    ros::Publisher  posePublisher_;
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener    tfListener_;

    bool initialized_;
    double totalDuration_;
    int scansCount_;
    PMScan * prevPMScan_;
    btTransform prevWorldToBase_;

    btTransform baseToLaser_;
    btTransform laserToBase_;

    // **** parameters

    int    minValidPoints_;
    int    searchWindow_;
    double maxError_;
    int    maxIterations_;
    double stopCondition_;
    bool   publishTf_;
    bool   publishPose_;
    bool   useOdometry_;

    std::string worldFrame_;
    std::string baseFrame_;
    std::string laserFrame_;

    bool initialize(const sensor_msgs::LaserScan& scan);
    void scanCallback (const sensor_msgs::LaserScan& scan);
    void getCurrentEstimatedPose(btTransform& worldToBase, 
                                 const sensor_msgs::LaserScan& scanMsg);
    void publishTf(const btTransform& transform, 
                   const ros::Time& time);
    void publishPose(const btTransform& transform);

    void rosToPMScan(const sensor_msgs::LaserScan& scan, 
                     const btTransform& change,
                           PMScan* pmScan);
    void pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t);
    void tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose);

  public:

    PSMNode();
    virtual ~PSMNode();
};

#endif // POLAR_SCAN_MATCHER_PSM_NODE_H
