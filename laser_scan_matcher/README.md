![attachment:laser_scan_matcher.png](doc/laser_scan_matcher.png)

## Details

The laser_scan_matcher package is an
incremental laser scan registration tool. The package allows to scan
match between consecutive
[sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
messages, and publish the estimated position of the laser as a
[geometry_msgs/msg/Pose2D](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Pose2D.html)
or a [tf](https://index.ros.org/p/tf) transform.

The package can be used without any odometry estimation provided by
other sensors. Thus, it can serve as a stand-alone odometry estimator.
Alternatively, you can provide several types of odometry input to
improve the registration speed and accuracy.

## Input

### Supported message types

The laser_scan_matcher can operate
using
[sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
messages or
[sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)
messages. When using
[sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html),
make sure they have no `nan` values.

### Scan prediction

While the laser_scan_matcher can
operate by just using scan data, we can speed up the scan registration
process by providing a guess for the current position of the sensor
every time a new scan message arrives. When no guess is available, a
reasonable (and widely-used) assumption is that the sensor didn't move
(zero-velocity model). Below is a list of inputs that
laser_scan_matcher accepts:

1\. **IMU** :An estimation for the change of the orientation angle
(`delta-theta`) of the robot in the form of a
[sensor_msgs/msg/IMU](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/IMU.html)
message. We are assuming that the `yaw` component of the IMU message
corresponds to the orientation of the robot. Thus, we don't really need
a full 6DoF IMU sensor - a cheap 1-axis gyro will work as well, as long
as its output is packed as an IMU message. The required topic is
`imu/data`.

2\. **Wheel odometry**: An estimation for the change of x-, y-, and
orientation angle of the robot from an odometric sensor such as wheel
encoders. The required topic is `odom`.

3\. **Constant velocity model**: Assumes the robot moved based on an
estimate of the robot's velocity. The velocity estimate can be obtained
from an external sensor, or by derivating and filtering the output of
the scan matcher itself. The required topic is `vel`. For an example
of how to use a simple filter to achieve this, check out
[Alpha-beta tracking for scan matching predictions](AlphaBetaTracking).

4\. **Zero-velocity model**: Don't use any prediction, ie, assume that
the robot stayed in the same place.

We can use combinations of the above such as IMU together with wheel
odometry or IMU together with alpha beta tracking. When several
prediction modes are enabled, the priority is IMU > Odometry >
Constant Velocity > Zero Velocity.

**What should I use in practice?**

IMU and (to some extent) wheel odometry inputs significantly improve
convergence speed for rotational motion. The addition of an IMU input is
thus highly recommended.

Alpha-beta tracking can lead to a significant speed up when the
performance of the scan matcher is stable, but might result in weird
behavior for highly dynamic environments or environments with poor
features. We recommend enabling it and determining empirically if it is
useful for your environment.

### Keyframes vs frame-to-frame scan matching

In the classical frame-to-frame laser odometry, each laser scan is
compared to the previous scan. The transformation between the two is
aggregated over time to calculate the position of the robot in the fixed
frame.

Some noise in the scans is inevitable. Thus, even for a robot standing
still, the incremental transformations might be non-zero. This could
result in a slow drift of the pose of the robot.

To alleviate this, we implement keyframe-based matching. The change in
pose is calculated between the current laser scan and a "keyframe"
scan. The keyframe scan is updated after the robot moves a certain
distance. Thus, if the robot is standing still, the keyframe scan will
not change, and the pose will remain more drift free.

Setting the tolerance for updating the keyframe can be achieved via the
`kf_dist_linear` and `kf_dist_angular` parameters. Their default
values give a more robust performance, both while standing still and
moving.

To change the scan_matching mode back to the classical frame-to-frame,
the user can simply set either of the two thresholds to zero.

## Example

You can run the laser_scan_matcher on
a pre-recorded bag file that comes with the package. First, make sure
you have the [scan_tools](https://index.ros.org/p/scan_tools) stack
downloaded and installed by following the
[instruction instructions](https://index.ros.org/p/scan_tools).



```shell
roslaunch laser_scan_matcher demo.launch
```



You should see a result similar to the video below. The video shows
tracking the position of a Hokuyo laser as it is being carried freely
around a room. The pose is determined entirely by the scan matcher - no
additional odometry is provided.

[Youtube](https://www.youtube.com/watch?v=sIMM73Was74)

## ROS API

Two drivers are available: `laser_scan_matcher_nodelet` and
`laser_scan_matcher_node`. Their parameters and topics are identical.

### Topics

#### Subscribed Topics

- `scan`
  ([sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)):
  Scans from a laser range-finder
- `cloud`
  ([sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)):
  Scans in point cloud form
- `imu/data`
  ([sensor_msgs/msg/Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html)):
  Imu messages, used for theta prediction. Only used if `use_imu` is
  set to `true`.
- `odom`
  ([nav_msgs/msg/Odometry](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Odometry.html)):
  Odometry messages, used for x-, y-, and theta prediction. Only used if
  `use_odom` is set to `true`.

#### Published Topics

- `pose2D`
  ([geometry_msgs/msg/Pose2D](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Pose2D.html)):
  The pose of the base frame, in some fixed (world) frame.

### Transforms

#### Required tf Transforms

- `base_link` → `laser`: the pose of the laser in the base frame.

#### Provided tf Transforms

- `world` → `base_link`: the pose of the robot base in the world frame.
  Only provided when `publish_tf` is enabled.

### Parameters

#### Coordinate frames

- `~fixed_frame` (`string`, default: `"world"`): the fixed frame
- `~base_frame` (`string`, default: `"base_link"`): the base frame
  of the robot

#### Motion prediction

- `~use_imu` (`bool`, default: `true`): Whether to use an imu for the
  theta prediction of the scan registration. Requires input on
  `/imu/data` topic.
- `~use_odom` (`bool`, default: `true`): Whether to use wheel odometry
  for the x-, y-, and theta prediction of the scan registration.
  Requires input on `odom` topic.
- `~use_vel` (`bool`, default: `false`): Whether to use constant
  velocity model for the x-, y-, and theta prediction of the scan
  registration. Requires input on `vel` topic.

#### Pointcloud input

Parameters when using
[sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)
instead of
[sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
messages.

- `~use_cloud_input` (`bool`, default: `false`): Whether to subscribe
  to a `/cloud` topic with
  [sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)
  messages instead of a `/scan` topic with
  [sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
  messages.
- `~cloud_range_min` (`double`, default: `0.1`): The minimum range of
  the sensor, if using
  [sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)
  messages. Not needed if using
  [sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
  messages.
- `~cloud_range_max` (`double`, default: `50.0`): The maximum range of
  the sensor, if using
  [sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)
  messages. Not needed if using
  [sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
  messages.

#### Keyframes

Parameters for setting up keyframe-scan based registration. Using the
default values, the keyframe is updated when the sensor moves 10 cm or
10 degrees. Setting either of these to zero will reduce to
frame-to-frame scan matching. Keeping them at default levels should
reduce drift while robot is stationary.

- `~kf_dist_linear` (`double`, default: `0.10`): What distance the fixed
  frame needs to move before updating the keyframe scan (in meters)
- `~kf_dist_angular` (`double`, default: `0.175`): What angle the fixed
  frame needs to move before updating the keyframe scan (in radians).

#### Output

- `~publish_tf` (`bool`, default: `true`): whether to publish scan
  matcher's estimation for the position of the base frame in the world
  frame as a transform.
- `~publish_pose` (`bool`, default: `true`): whether to publish scan
  matcher's estimation for the position of the base frame in the world
  frame as a
  [geometry_msgs/msg/Pose2D](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Pose2D.html)
- `~publish_pose_stamped` (`bool`, default: `false`): whether to
  publish scan matcher's estimation for the position of the base frame
  in the world frame as a
  [geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)

#### Scan matching

- `~max_iterations` (`int`, default: `10`): Maximum ICP cycle iterations
- `~max_correspondence_dist` (`double`, default: `0.3`): Maximum
  distance for a correspondence to be valid
- `~max_angular_correction_deg` (`double`, default: `45.0`): Maximum
  angular displacement between scans, in degrees
- `~max_linear_correction` (`double`, default: `0.50`): Maximum
  translation between scans (m)
- `~epsilon_xy` (`double`, default: `0.000001`): A threshold for
  stopping (m)
- `~epsilon_theta` (`double`, default: `0.000001`): A threshold for
  stopping (rad)
- `~outliers_maxPerc` (`double`, default: `0.90`): Percentage of
  correspondences to consider: if 0.90, always discard the top 10% of
  correspondences with more error

#### Scan matching (advanced)

- `~sigma` (`double`, default: `0.010`): Noise in the scan (m) (Not sure
  if changing this has any effect in the current implementation)
- `~use_corr_tricks` (`int`, default: `1`): If 1, use smart tricks for
  finding correspondences (see paper).
- `~restart` (`int`, default: `0`): Restart: If 1, restart if error is
  over threshold
- `~restart_threshold_mean_error` (`double`, default: `0.01`): Restart:
  Threshold for restarting
- `~restart_dt` (`double`, default: `1.0`): Restart: displacement for
  restarting. (m)
- `~restart_dtheta` (`double`, default: `0.1`): Restart: displacement
  for restarting. (rad)
- `~clustering_threshold` (`double`, default: `0.25`): Max distance for
  staying in the same clustering
- `~orientation_neighbourhood` (`int`, default: `10`): Number of
  neighbour rays used to estimate the orientation
- `~use_point_to_line_distance` (`int`, default: `1`): If 0, it's
  vanilla ICP
- `~do_alpha_test` (`int`, default: `0`): If 1, discard correspondences
  based on the angles
- `~do_alpha_test_thresholdDeg` (`double`, default: `20.0`): Discard
  correspondences based on the angles - threshold angle, in degrees
- `~outliers_adaptive_order` (`double`, default: `0.7`): Parameters
  describing a simple adaptive algorithm for discarding. 1) Order the
  errors. 2) Choose the percentile according to
  `outliers_adaptive_order` (if it is 0.7, get the 70% percentile). 3)
  Define an adaptive threshold multiplying `outliers_adaptive_mult`
  with the value of the error at the chosen percentile. 4) Discard
  correspondences over the threshold. This is useful to be conservative;
  yet remove the biggest errors.
- `~outliers_adaptive_mul` (`double`, default: `2.0`): Parameters
  describing a simple adaptive algorithm for discarding. 1) Order the
  errors. 2) Choose the percentile according to
  `outliers_adaptive_order` (if it is 0.7, get the 70% percentile). 3)
  Define an adaptive threshold multiplying `outliers_adaptive_mult`
  with the value of the error at the chosen percentile. 4) Discard
  correspondences over the threshold. This is useful to be conservative;
  yet remove the biggest errors.
- `~do_visibility_test` (`int`, default: `0`): If you already have a
  guess of the solution, you can compute the polar angle of the points
  of one scan in the new position. If the polar angle is not a monotone
  function of the readings index, it means that the surface is not
  visible in the next position. If it is not visible, then we don't use
  it for matching.
- `~outliers_remove_doubles` (`int`, default: `1`): If 1, no two points
  in laser_sens can have the same correspondence
- `~do_compute_covariance` (`int`, default: `0`): If 1, computes the
  covariance of ICP using the method <http://purl.org/censi/2006/icpcov>
  (Changing this has no effect in the current implementation)
- `~debug_verify_trick` (`int`, default: `0`): If 1, checks that
  find_correspondences_tricks gives the right answer
- `~use_ml_weights` (`int`, default: `0`): If 1, the field
  'true_alpha' (or 'alpha') in the first scan is used to compute the
  incidence beta, and the factor (1/cos\^2(beta)) used to weight the
  correspondence. (Changing this has no effect in the current
  implementation)
- `~use_sigma_weights` (`int`, default: `0`): If 1, the field
  'readings_sigma' in the second scan is used to weight the
  correspondence by 1/sigma\^2 (Not sure if changing this has any effect
  in the current implementation)

## References

\[1\] A. Censi, "An ICP variant using a point-to-line metric"
Proceedings of the IEEE International Conference on Robotics and
Automation (ICRA), 2008

## Bug Reports & Feature Requests

We appreciate the time and effort spent submitting bug reports and
feature requests.

Please submit your tickets through
[github](https://github.com/ccny-ros-pkg/scan_tools/issues/) (requires
github account) or by emailing the maintainers.
