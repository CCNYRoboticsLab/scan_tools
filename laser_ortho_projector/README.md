![attachment:laser_ortho_projector.png](doc/laser_ortho_projector.png)

## Usage

The package is intended as an intermediary step to transform laser data,
before passing it to another node that performs scan-matching (for
example,
[laser_scan_matcher](https://index.ros.org/p/laser_scan_matcher) or
[slam_gmapping](https://index.ros.org/p/slam_gmapping)). It works under
the assumption that the environment is rectilinear in the z-dimension,
meaning that obstacles look the same, regardless of the height where
they are observed. This assumption typically holds for indoor spaces.

The orthogonal projector can be used for vehicles with varying roll-,
pitch-, and z-pose that travel in indoor environments.

Alternatively, it can be used for vehicles that travel in 2D, but have a
laser scanner that is mounted at an angle to the horizontal plane.

## Details

Laser scan data is often used to calculate the motion of mobile robots
in static environments. Most scan-matching implementations assume that
the robot only moves in the xy plane, and calculate the translation and
rotation that best maps two sets of points from consecutive laser scans
onto each other.

Attempting to apply this technique directly to robots in 3D presents two
key challenges. The additional degrees of freedom (z, pitch, roll)
impose a greater computational cost. More importantly, the data obtained
by statically-mounted planar laser scanners is not sufficient to
correctly determine the change in the 3D pose.

In most indoor environments, obstacles look the same regardless of the
height. If we know the 3D orientation of the laser scanner, we can
reduce the problem of 3D scan matching to 2D by projecting the scans
onto the xy-plane.

We begin by calculating the Cartesian coordinates of the points in the
laser scan, in a base frame of the robot. Next, we discard the z-values
of the points, to obtain the set of points in the horizontal plane.

We also calculate a transformation between the world frame and a
orthogonally-projected base frame of the robot (labeled `/base_ortho`
in the figure above), and publish it as a
[tf](https://index.ros.org/p/tf) transform.

Note that projecting the scan from an arbitrary roll and pitch results
in points that are no longer equal angles apart. Thus, we pubish the
projected scan as a
[pcl/msg/PointCloudXYZ](https://docs.ros.org/en/rolling/p/pcl/msg/PointCloudXYZ.html).
The point cloud is published in the base_ortho frame of reference.

## Example

You can run the
laser_ortho_projector on a
pre-recorded bag file that comes with the package. First, make sure you
have the [scan_tools](https://index.ros.org/p/scan_tools) stack
downloaded and installed by following the instructions
[here](https://index.ros.org/p/scan_tools).

Next, make sure you have the necessary tools installed:



```shell
rosmake rviz tf rosbag
```



Finally, run the demo:



```shell
roslaunch laser_ortho_projector demo.launch
```



You should see the input laser scan and the orthogonally projected scan
displayed in [rviz](https://index.ros.org/p/rviz), similar to the video
below:

[Youtube](https://www.youtube.com/watch?v=jHsdwQbqc8Y)

## Nodes & Nodelets

Two drivers are available: `laser_ortho_projector_node` and
`laser_ortho_projector_nodelet`. Their parameters and topics are
identical.

### laser_ortho_projector_node / laser_ortho_projector_nodelet

The `laser_ortho_projector_node` takes in
[sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
messages and outputs their orthogonal projections in the xy-plane.

#### Subscribed Topics

- `scan`
  ([sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)):
  Scans from a laser range-finder
- `imu`
  ([sensor_msgs/msg/IMU](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/IMU.html)):
  Optional IMU data for the orientation of the `base_link` frame. If
  not provided, the orientation of the `base_link` frame needs to be
  provided as a transform using `tf`.

#### Published Topics

- `~cloud_ortho`
  ([sensor_msgs/msg/PointCloud](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud.html)):
  The projected laser scan, in Cartesian coordinates, in the
  `base_ortho` frame (orthogonal projection of the `base_link`
  frame.

#### Parameters

- `~fixed_frame` (`string`, default: `"/world"`): the name of the
  fixed frame
- `~base_frame` (`string`, default: `"base_link"`): the name of the
  vehicle base frame
- `~publish_tf` (`bool`, default: `false`): whether to publish the tf
  between the `fixed_frame` and the `base_ortho` frames as a
  `tf`>>.
- `~use_imu` (`bool`, default: `true`): whether to use IMU data for
  the orientation of the `base_link` frame. If set to `false`, the
  orientation of the `base_link` frame needs to be provided as a
  transform using `tf`.

#### Required tf Transforms

- `base_link` → `laser`: the pose of the laser in in reference to the
  base frame
- `world` → `base_link`: the pose of the bae link in the fixed frame.
  Not needed if IMU data is provided.

#### Provided tf Transforms

- `world` → `base_ortho`: the pose of the projected base frame in the
  world frame

## Bug Reports & Feature Requests

We appreciate the time and effort spent submitting bug reports and
feature requests.

Please submit your tickets through
[github](https://github.com/ccny-ros-pkg/scan_tools/issues/) (requires
github account) or by emailing the maintainers.
