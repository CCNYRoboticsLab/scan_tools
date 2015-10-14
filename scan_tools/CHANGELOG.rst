^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scan_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2015-10-14)
------------------
* [feat] Released into ROS Indigo and Jade
* [feat] Update gmapping demo as well.
* [feat] added support for IMU message for orientation in laser_ortho_projector. The imu topic is imu/data* [feat] removed vgf filter, added xy filtering. added vgf on point cloud input for scan_matcher, vgf on point cloud input for scan_matcher
* [feat] removing pose2dstamped message type, now using posestamped
* [feat] added polar scan matcher
* [feat] added imu option to PSM, made CSM and PSM look similar
* [feat] added demo dir to CSM, 
* [fix] rotation update check
* [fix] possible memory leak introduced by keyframe feature
* [fix] PSM launch file
* [doc] updated manifest of scan_to_cloud_converter
* [doc] README files to contain catkin instructions
* [doc] replaces tabs and fixes imu <-> odom topic comment
* [sys] Catkinization
* [sys] Moved from robotics.ccny.cuny.edu
* [sys] ROS Hydro compatible
* [sys] correct license and references to PSM and CSM
* [sys] Small tweaks to CMakeLists.txt and package.xml files
* [sys] renamed skip to step parameter in scan sparsifier
* [sys] cleaning
* [sys] Replace demo.vcg with a demo.rviz
* [sys] Small tweaks to CMakeLists.txt and package.xml files
* [sys] laser_scan_matcher: Remove CSM search in CMakeLists.txt
  Since the csm package exports the CFLAGS and LFLAGS, we do not need to
  explicitly search for the paths.
* [sys] launch files use rosbag api for fuerte
* [sys] imu topic renamed to imu/data
* Contributors: Ivan Dryanovski, Miguel Sarabia, Daniel Axtens, Kartik Mohta, Miguel Sarabia, Stephan Wirth, Enrique Fernandez, Isaac I.Y. Saito
