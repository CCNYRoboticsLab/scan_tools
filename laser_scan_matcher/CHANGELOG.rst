^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_scan_matcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [feat] Update gmapping demo as well.
* [feat] removed vgf filter, added xy filtering. added vgf on point cloud input for scan_matcher, vgf on point cloud input for scan_matcher
* [feat] removing pose2dstamped message type, now using posestamped
* [fix] rotation update check
* [fix] possible memory leak introduced by keyframe feature
* [doc] replaces tabs and fixes imu <-> odom topic comment
* [sys] cleaning
* [sys] Replace demo.vcg with a demo.rviz
* [sys] Small tweaks to CMakeLists.txt and package.xml files
* [sys] Catkinization: laser_scan_matcher (csm integrated as external project
* [sys] laser_scan_matcher: Remove CSM search in CMakeLists.txt
  Since the csm package exports the CFLAGS and LFLAGS, we do not need to
  explicitly search for the paths.
* [sys] launch files use rosbag api for fuerte
* [sys] imu topic renamed to imu/data
* Contributors: Daniel Axtens, Ivan Dryanovski, Kartik Mohta, Miguel Sarabia, Stephan Wirth, enriquefernandez
