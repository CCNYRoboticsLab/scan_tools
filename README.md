Laser scan tools for ROS
===================================

Overview
-----------------------------------

Laser scan processing tools. The meta-package contains:

 * `laser_ortho_projector`: calculates orthogonal projections of LaserScan messages
    - License: BSD-3-Clause
 
 * `laser_scan_matcher`: an incremental laser scan matcher, using Andrea Censi's Canonical 
Scan Matcher (CSM) implementation [1].
    - License: BSD-3-Clause, LGPL
    - Note: CSM is LGPL-3.0 licensed and depends on either [GSL](https://www.gnu.org/software/gsl/), which is GPL-3.0 or [eigen](https://eigen.tuxfamily.org/), which is MPL-2.0, depending on the version of CSM.

 * `laser_scan_sparsifier`: takes in a LaserScan message and sparsifies it
    - License: BSD-3-Clause

 * `laser_scan_splitter`:  takes in a LaserScan message and splits 
it into a number of other LaserScan messages 
    - License: BSD-3-Clause

 * `ncd_parser`: reads in .alog data files from the New College Dataset [2]
and broadcasts scan and odometry messages to ROS.
    - License: BSD-3-Clause

 * `polar_scan_matcher`: used to produce a pose estimate for a robot when no odometry is available.
    - License: GPL-2.0

 * `scan_to_cloud_converter`: converts LaserScan to PointCloud messages.
    - License: BSD-3-Clause

Installing
-----------------------------------

### Prerequisite

* ROS is installed
  * [1-liner ROS Indigo installation on Ubuntu](http://wiki.ros.org/ROS/Installation/TwoLineInstall)
* `apt-get install python-wstool`

### From binary (RECOMMENDED)

```
apt-get install ros-%ROS_DISTRO%-scan-tools

apt-get install ros-indigo-scan-tools        (Indigo)
```

### From source ###

Following is an example with ROS Indigo.

1. Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and navigate to its source directory (e.g. `~/catkin_ws/src`).

2. In your Catkin workspace, download source and build with the following commands.

```
cd ~/catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ccny-ros-pkg/scan_tools/indigo/.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro indigo -r -y
catkin_make                (or any build commands available in ROS, e.g. `catkin build`)
source devel/setup.bash
```

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
 
