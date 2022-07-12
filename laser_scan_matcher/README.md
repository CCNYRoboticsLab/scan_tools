# General Parameters

| Name  | Type | Description | Default | Min | Max | Reconfigurable |
|----|----|----|----|----|----|----|
| `base_frame` | `string` | Which frame to use for the robot base | "base_link" | | |✗|
| `degeneracy_check` | `bool` | Check for degeneracy of matches along an axis | `false` | | |✓|
| `degeneracy_cov_ramp` | `double` | Power to apply to `[0,1]` degeneracy metric prior to scaling for covariance | `5.0` | `1.0` | `10.0` |✓|
| `degeneracy_cov_scale` | `double` | Scaling to apply to degeneracy metric for covariance along degenerate axis | `1.0` | `0.0` | `100.0` |✓|
| `degeneracy_cov_offset` | `double` | Offset to apply to position covariance along degenerate axis | `0.0` | `0.0` | `10.0` |✓|
| `heading_cov_scale` | `double` | Scaling to apply to ICP derived heading covariance | `1.0` | `0.0` | `1e8` |✓|
| `heading_cov_offset` | `double` | Offset to apply to ICP derived heading covariance | `0.0` | `0.0` | `10.0` |✓|
| `min_travel_distance` | `double` | Distance in meters to trigger a new keyframe | `0.5` | `0.0` | `10.0` |✓|
| `min_travel_heading` | `double` | Angle in degrees to trigger a new keyframe. | `30.0` | `0.0` | `180.0` |✓|
| `odom_frame` | `string` | Which frame to track odometry in | "odom" | | |✗|
| `publish_tf` | `bool` | Whether to publish tf transform from `odom_frame` to `base_frame` | `true` | | |✗|
| `tf_timeout` | `double` | TF timeout in seconds | `0.1` | `0.0` | `10.0` |✓|
| `xy_cov_scale` | `double` | Scaling to apply to ICP derived xy position covariance | `1.0` | `0.0` | `1e8` |✓|
| `xy_cov_offset` | `double` | Offset to apply to ICP derived xy position covariance | `0.0` | `0.0` | `10.0` |✓|

# CSM Parameters

| Name  | Type | Description | Default | Min | Max | Reconfigurable |
|----|----|----|----|----|----|----|
| `alpha_test_threshold` | `double` | Threshold angle to discard correspondences, in degrees | `20.0` | `0.0` | `90.0` |✓|
| `clustering_threshold` | `double` | Max distance, in meters, for staying in the same clustering | `0.25` | `0.0` | `1.0` |✓|
| `debug_verify_tricks` | `bool` | Checks that find_correspondences_tricks gives the right answer | `false` | | |✓|
| `do_alpha_test` | `bool` | Discard correspondences based on the angles | `false` | | |✓|
| `do_compute_covariance` | `bool` | Compute the covariance of the ICP | `true` | | |✓|
| `do_visibility_test` | `bool` | Use visibility test trick for matching | `false` | | |✓|
| `epsilon_theta` | `double` | A threshold for stopping, in radians | `1e-6` | `1e-10` | `1e-1` |✓|
| `epsilon_xy` | `double` | A threshold for stopping, in meters | `1e-6` | `1e-10` | `1e-1` |✓|
| `max_angular_correction_deg` | `double` | Max distance, in meters, for a correspondence to be valid | `0.3` | `0.0` | `10.0` |✓|
| `max_correspondence_dist` | `double` | Max angular displacement, in degrees, between scans | `45` | `0.0` | `90.0` |✓|
| `max_iterations` | `int` | Max ICP cycle iterations | `10` | `1` | `100` |✓|
| `max_linear_correction` | `double` | Max translation, in meters, between scans | `0.5` | `0.0` | `10.0` |✓|
| `orientation_neighbourhood` | `int` | Number of neighbour rays used to estimate the orientation | `20` | `0` | `100` |✓|
| `outliers_adaptive_mult` | `double` | Threshold multiplier for error at chosen percentile | `2.0` | `0.0` | `10.0` |✓|
| `outliers_adaptive_order` | `double` | Percentile `[0, 1]` to use for adaptive outlier detection | `0.7` | `0.0` | `1.0` |✓|
| `outliers_max_perc` | `double` | Percentage `[0, 1]` of correspondences to consider with lowest error | `0.9` | `0.0` | `1.0` |✓|
| `outliers_remove_doubles` | `bool` | Do not allow two different correspondences to share a point | `true` | | |✓|
| `restart` | `bool` | Restart if error is over threshold | `false` | | |✓|
| `restart_dt` | `double` | Displacement, in meters, for restarting | `1.0` | `0.0` | `10.0` |✓|
| `restart_dtheta` | `double` | Displacement, in radians, for restarting | `0.1` | `0.0` | `1.57` |✓|
| `restart_threshold_mean_error` | `double` | Threshold, in meters, for restarting | `0.01` | `0.0` | `1.0` |✓|
| `sigma` | `double` | Noise in the scan, in meters | `0.01` | `0.0` | `1.0` |✓|
| `use_corr_tricks` | `bool` | Use smart tricks for finding correspondences | `true` | | |✓|
| `use_ml_weights` | `bool` | Use the 'true_alpha' or 'alpha' field to weight the impact of each correspondence | `false` | | |✓|
| `use_point_to_line_distance` | `bool` | Use point-to-line ICP.  If false, it's vanilla ICP | `true` | | |✓|
| `use_sigma_weights` | `bool` | Use the 'readings_sigma' field of the second scan to weight the correspondence | `false` | | |✓|
