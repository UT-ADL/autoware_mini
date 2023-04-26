# Global planning


## lanelet2_global_planner

A ROS node that implements a global planner for autonomous vehicles based on Lanelet2 map. The node subscribes to the `move_base_simple/goal` topic and publishes the global path as a sequence of waypoints to the `global_path` topic. The node uses the current position of the vehicle from the `localization/current_pose` topic and generates the global path to the goal position by finding the shortest path in the Lanelet2 map. Multiple goal points can be added.


#### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `output_frame` | `string` | `"map"` | The name of the output frame for the generated path |
| `distance_to_goal_limit` | `float` | `2.0` | The minimum distance from the last waypoint to the goal position for the generated path |
| `distance_to_centerline_limit` | `float` | `5.0` | The maximum distance from the centerline for a waypoint on the generated path |
| `speed_limit` | `float` | `40.0` | The speed limit for the generated path |
| `wp_left_width` | `float` | `1.4` | The width of the left lane of the generated path |
| `wp_right_width` | `float` | `1.4` | The width of the right lane of the generated path |
| `nearest_neighbor_search` | `string` | `"kd_tree"` | The type of the nearest neighbor search algorithm used by the planner |


#### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | The goal position of the vehicle |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | The current position of the vehicle |
| `/cancel_route` | `std_msgs/Bool` | A boolean flag to cancel the existing global path |

#### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `global_path` | `autoware_msgs/Lane` | The generated global path |
| `target_lane_markers` | `visualization_msgs/MarkerArray` | The markers for the target lane (mainly for debugging purpose) |



## path_smoothing


A ROS node for smoothing global path using interpolation and optional speed adjustments based on deceleration limit, path radius and lateral acceleration.

#### Parameters

| Name | Type | Default Value | Description |
| --- | --- | --- | --- |
| `~waypoint_interval` | float | `1.0` | Distance between waypoints after path smoothing (m)|
| `~adjust_speeds_in_curves` | bool | `True` | Whether to adjust speeds based on the curvature of the path. |
| `~adjust_speeds_using_deceleration` | bool | `True` | Whether to adjust speeds based on maximum deceleration. |
| `~adjust_endpoint_speed_to_zero` | bool | `True` | Whether to adjust the speeds at the end of the path to decelerate to zero. |
| `~speed_deceleration_limit` | float | `1.0` | Deceleration limit used in speed adjustment (m/s2) |
| `~speed_averaging_window` | int | `21` | Number of points used to calculate average speed. |
| `~radius_calc_neighbour_index` | int | `4` | Index of points (+/- from center point) used to calculate radius for the path. |
| `~lateral_acceleration_limit` | float | `3.0` | Maximum allowed lateral acceleration limit (m/s2) |
| `~output_debug_info` | bool | `False` | Whether to output debug information. Debug information will draw graphs using a function from helpers. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/global_path` | `autoware_msgs/Lane` | Subscribes to the global path to be smoothed. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/smoothed_path` | `autoware_msgs/Lane` | Publishes the smoothed path with equal distances between waypoints |
