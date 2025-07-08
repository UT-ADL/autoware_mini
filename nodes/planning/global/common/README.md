# Planning - global/common


## goal_publisher

ROS node that publishes goal poses from existing YAML file to be listed in Carla RViz plugin as scenarios.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~goals_file` | string | `-` | Path to the YAML file containing goal positions. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Goal pose for the vehicle to navigate to. |
| `global_path` | `autoware_mini/Path` | Global path calculated by the planning system. Used to track the status of navigation to the goal. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/carla/available_scenarios` | `carla_ros_scenario_runner_types/CarlaScenarioList` | List of available scenarios defined in the goals file. Subscribed by Carla RViz plugin. |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Goal pose for the vehicle to navigate to. |
| `/scenario_runner/status` | `carla_ros_scenario_runner_types/CarlaScenarioRunnerStatus` | Status of navigation to the current goal (STOPPED, STARTING, RUNNING). |


#### Services

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/scenario_runner/execute_scenario` | `carla_ros_scenario_runner_types/ExecuteScenario` | Service to publish a predefined goal. Called by Carla RViz plugin. |


## path_smoothing

ROS node that smooths global paths by interpolating waypoints, adjusting speeds in curves, and ensuring proper deceleration profiles.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `waypoint_interval` | float | `1.0` | Distance between waypoints in the smoothed path in meters. |
| `~adjust_speeds_in_curves` | bool | `True` | Whether to adjust speeds in curves based on lateral acceleration limit. |
| `~adjust_speeds_using_deceleration` | bool | `True` | Whether to adjust speeds along the path to respect deceleration limits. |
| `~adjust_endpoint_speed_to_zero` | bool | `True` | Whether to set the endpoint speed to zero and adjust preceding waypoints. |
| `default_deceleration` | float | `1.0` | Default deceleration value in m/s² used for speed adjustment. |
| `~speed_averaging_window` | int | `21` | Window size for averaging speeds along the path. |
| `~radius_calc_neighbour_index` | int | `8` | Index of neighbors used for radius calculation in curve detection. |
| `~lateral_acceleration_limit` | float | `1.5` | Maximum allowed lateral acceleration in m/s² for curve speed calculation. |
| `~output_debug_info` | bool | `False` | Whether to output debug information and plots. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `lane_change_global_path` | `autoware_mini/Path` | Raw global path to be smoothed. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `global_path` | `autoware_mini/Path` | Smoothed global path with adjusted waypoint positions and speeds. |

