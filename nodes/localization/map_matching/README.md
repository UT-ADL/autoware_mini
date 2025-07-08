# Localization - Map Matching

## lane_boundary_matcher

ROS node that matches lane boundaries detected by a camera-based system (comma.ai openpilot) with map lane boundaries to improve localization accuracy.

#### Parameters

| Name | Type | Default Value | Description                                                                           |
| ----- | ----- | ------------- |---------------------------------------------------------------------------------------|
| `~lookahead_distance` | float | `3.5` | Distance ahead of the vehicle to consider for lane boundary matching (meters).        |
| `~enable_height_correction` | bool | `False` | Whether to correct vehicle height based on map data.                                  |
| `~x_correction_treshold` | float | `0.5` | Maximum allowed x-axis correction (meters). Corrections larger than this are ignored. |
| `~y_correction_treshold` | float | `0.5` | Maximum allowed y-axis correction (meters). Corrections larger than this are ignored. |
| `~probability_treshold` | float | `0.4` | Minimum confidence level required for lane boundary detection to be used.             |
| `~transform_timeout` | float | `0.06` | Timeout in seconds for waiting for a transform to become available.                   |
| `~openpilot_delay_compensation` | float | `0.1` | Time in seconds to compensate for openpilot detection delay.                          |
| `~alpha` | float | `0.03` | Learning rate for exponential moving average filter used to smooth corrections.       |
| `~no_correction_weight` | float | `0.4` | Weight applied for smoothing when no correction is made.                              |

#### Subscribed Topics

| Name | Type | Description                                     |
| ----- | ----- |-------------------------------------------------|
| `current_pose_gnss` | `geometry_msgs/PoseStamped` | Current vehicle pose from GNSS.                 |
| `/planning/lanelet2_global_path` | `autoware_mini/Path` | Global path with lane boundary information.     |
| `/openpilot/lane_lines` | `vehicle_platform/Float32MultiArrayStamped` | Lane line detections from the openpilot system. |
| `/openpilot/lane_line_probs` | `vehicle_platform/Float32MultiArrayStamped` | Probability values for lane line detections from the openpilot system.   |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `current_pose` | `geometry_msgs/PoseStamped` | Corrected vehicle pose by applying map matching adjustments. |
| `lane_boundary_matcher_markers` | `visualization_msgs/MarkerArray` | Visualization markers for detected and map lane boundaries. |
| `/dashboard/gnss_corrections_detailed` | `jsk_rviz_plugins/OverlayText` | Text overlay showing current correction values for debugging. |

#### Transforms

| Parent | Child | Description |
| ----- | ----- | ------------ |
| `map` | `map_gnss` | Transformation that encodes the correction between the map frame and the GNSS-based map frame. |
