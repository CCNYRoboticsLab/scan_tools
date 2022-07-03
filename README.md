Laser scan tools for ROS
===================================

Overview
-----------------------------------

Laser scan processing tools. The meta-package contains:

 * `laser_scan_matcher`: an incremental laser scan matcher, using Andrea Censi's Canonical 
Scan Matcher [1] implementation.

 * `scan_to_cloud_converter`: converts LaserScan to PointCloud messages.

License
-----------------------------------

This repo is licensed with [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause), but depends on the version
of [CSM](https://github.com/AndreaCensi/csm) which uses [eigen](https://eigen.tuxfamily.org/). CSM itself
is [LGPL-3.0](https://opensource.org/licenses/LGPL-3.0), while eigen is [MPL-2.0](https://opensource.org/licenses/MPL-2.0)


More info
-----------------------------------

http://wiki.ros.org/scan_tools

References
-----------------------------------
 [1] A. Censi, "An ICP variant using a point-to-line metric" Proceedings of the 
IEEE International Conference on Robotics and Automation (ICRA), 2008

 [2] M. Smith, I. Baldwin, W. Churchill, R. Paul, and P. Newman, 
The new college vision and laser data set, International Journal for Robotics 
Research (IJRR), vol. 28, no. 5, pp. 595599, May 2009.
 
