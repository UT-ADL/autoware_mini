# Control

This folder contains classical controllers - nodes that do waypoint following. The source of the waypoints can be the waypoint file or outputs from the local or global planner.

## pure_pursuit_controller

A ROS node which implements the pure pursuit control algorithm. The node subscribes to the topic `/planning/local_path` to get the planned path and subscribes to the topics `/localization/current_pose` and `/localization/current_velocity` to get the current pose and velocity of the vehicle. It publishes to the topic `vehicle_cmd` the vehicle commands such as steering angle and velocity. The implementation is based on the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

#### Parameters

| Name                           | Type   | Default | Description                                                            |
| ------------------------------ | ------ | ------- | ---------------------------------------------------------------------- |
| `/vehicle/wheel_base`          | double | `2.789` | The distance in meters between the front and rear axles of the vehicle. |
| `/planning/default_acceleration`| double | `1.0`   | Default acceleration (in m/s^2).                                      |
| `/planning/default_deceleration`| double | `1.0`   | Default deceleration (in m/s^2).                                      |
| `/planning/max_deceleration`   | double | `10.0`  | Maximum deceleration (in m/s^2).                                      |
| `heading_angle_limit`          | double | `90.0`  | The maximum heading angle difference in degrees.                        |
| `lateral_error_limit`          | double | `2.0`   | The maximum lateral error in meters.                                   |
| `blinker_lookahead_time`       | double | `3.0`   | Lookahead time for blinker state (multiplied with velocity).           |
| `blinker_min_lookahead_distance` | double | `14.0` | Min lookahead distance to get blinker state (m).                      |
| `stopping_speed_limit`         | double | `1.0`   | Speed threshold below which the vehicle is considered stopped (m/s).   |
| `~lookahead_time`              | double | `1.2`   | The time in seconds to plan ahead for the lookahead distance.          |
| `~min_lookahead_distance`      | double | `6.0`   | The minimum lookahead distance to maintain from the current position.  |
| `~simulate_cmd_delay`          | double | `0.3`   | Time (s) to project ego vehicle forward to compensate for delay.       |
| `~publish_debug_info`          | bool   | `False` | Whether to publish debug information.                                  |

#### Subscribed Topics

| Name                           | Type                           | Description                                        |
| ------------------------------ | ------------------------------ | -------------------------------------------------- |
| `/planning/local_path`         | `autoware_mini/Path`           | The planned path.                                  |
| `/localization/current_pose`   | `geometry_msgs/PoseStamped`    | The current pose of the vehicle.                   |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | The current velocity of the vehicle.              |

#### Published Topics

| Name                  | Type                         | Description                                            |
| --------------------- | ---------------------------- | ------------------------------------------------------ |
| `vehicle_cmd`         | `autoware_mini/VehicleCmd`   | The vehicle commands (steering angle and velocity).    |
| `controller_markers`  | `visualization_msgs/MarkerArray` | If `publish_debug_info` is enabled: controller-specific visualization markers. |
| `controller_debug`    | `std_msgs/Float32MultiArray` | If `publish_debug_info` is enabled: debug data including processing time, heading information, errors, and target velocity. |

## stanley_controller

A ROS node that implements the Stanley control algorithm for lateral control. The node subscribes to the topic `/planning/local_path` to get the planned path and subscribes to the topics `/localization/current_pose` and `/localization/current_velocity` to get the current pose and velocity of the vehicle. It publishes to the topic `vehicle_cmd` the vehicle commands such as steering angle and velocity. The implementation is based on the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

#### Parameters

| Name                           | Type   | Default | Description                                                            |
| ------------------------------ | ------ | ------- | ---------------------------------------------------------------------- |
| `/vehicle/wheel_base`          | double | `2.789` | The distance in meters between the front and rear axles of the vehicle. |
| `/planning/default_acceleration`| double | `1.0`   | Default acceleration (in m/s^2).                                      |
| `/planning/default_deceleration`| double | `1.0`   | Default deceleration (in m/s^2).                                      |
| `/planning/max_deceleration`   | double | `10.0`  | Maximum deceleration (in m/s^2).                                      |
| `heading_angle_limit`          | double | `90.0`  | The maximum heading angle difference in degrees.                        |
| `lateral_error_limit`          | double | `2.0`   | The maximum lateral error in meters.                                   |
| `blinker_lookahead_time`       | double | `3.0`   | Lookahead time for blinker state (multiplied with velocity).           |
| `blinker_min_lookahead_distance` | double | `14.0` | Min lookahead distance to get blinker state (m).                      |
| `stopping_speed_limit`         | double | `1.0`   | Speed threshold below which the vehicle is considered stopped (m/s).   |
| `~cte_gain`                    | double | `0.4`   | Gain for cross-track error.                                           |
| `~simulate_cmd_delay`          | double | `0.3`   | Time (s) to project ego vehicle forward to compensate for delay.       |
| `~publish_debug_info`          | bool   | `False` | Whether to publish debug information.                                  |

#### Subscribed Topics

| Name                           | Type                           | Description                                        |
| ------------------------------ | ------------------------------ | -------------------------------------------------- |
| `/planning/local_path`         | `autoware_mini/Path`           | The planned path.                                  |
| `/localization/current_pose`   | `geometry_msgs/PoseStamped`    | The current pose of the vehicle.                   |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | The current velocity of the vehicle.              |

#### Published Topics

| Name                  | Type                         | Description                                            |
| --------------------- | ---------------------------- | ------------------------------------------------------ |
| `vehicle_cmd`         | `autoware_mini/VehicleCmd`   | The vehicle commands (steering angle and velocity).    |
| `controller_markers`  | `visualization_msgs/MarkerArray` | If `publish_debug_info` is enabled: controller-specific visualization markers. |
| `controller_debug`    | `std_msgs/Float32MultiArray` | If `publish_debug_info` is enabled: debug data including processing time, heading information, errors, and target velocity. |
