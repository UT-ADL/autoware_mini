# Control

This folder contains classical controllers - nodes that do waypoint following. The source of the waypoints can be the waypoint file or outputs from the local or global planner.


## pure_pursuit_follower

A ROS node which implements the pure pursuit control algorithm. The node subscribes to the topic `/planning/local_path` to get the planned path and subscribes to the topics `/localization/current_pose` and `/localization/current_velocity` to get the current pose and velocity of the vehicle. It publishes to the topic `/vehicle_cmd` the vehicle commands such as steering angle and velocity. The implementation is based on the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

#### Parameters

| Name                       | Type   | Default | Description                                                            |
| --------------------------| ------ | ------- | ---------------------------------------------------------------------- |
| `~planning_time`           | double | `2.0`   | The time in seconds to plan ahead for the lookahead distance.          |
| `~min_lookahead_distance`  | double | `6.0`   | The minimum lookahead distance to maintain from the current position.  |
| `/vehicle/wheel_base`      | double | `2.789` | The distance in meters between the front and rear axles of the vehicle. |
| `heading_angle_limit`      | double | `90.0`  | The maximum steering angle in degrees.                                 |
| `blinker_lookahead_time`   | double | `3.0`   | Lookahead time for blinker state (multiplied with velocity)            |
| `lateral_error_limit`      | double | `2.0`   | The maximum lateral error in meters.                                   |
| `~publish_debug_info`      | bool   | `False` | Whether to publish debug information.                                  |
| `~nearest_neighbor_search` | string | `"kd_tree"` | The algorithm used for nearest neighbor search (see sklearn.neighbors). |

#### Subscribed Topics

| Name                     | Type                           | Description                                        |
| ------------------------| ------------------------------| -------------------------------------------------- |
| `/planning/local_path`   | `autoware_msgs/Lane`           | The planned path.                                  |
| `/localization/current_pose`    | [`geometry_msgs/PoseStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | The current pose of the vehicle.                  |
| `/localization/current_velocity`| [`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)   | The current velocity of the vehicle.              |

#### Published Topics

| Name                  | Type                      | Description                                            |
| ----------------------| -------------------------| ------------------------------------------------------ |
| `vehicle_cmd`        | `autoware_msgs/VehicleCmd` | The vehicle commands (steering angle and velocity).    |
| `follower_markers`   | `visualization_msgs/MarkerArray` | if `publish_debug_info` is enabled: follower-specific visualization topic that helps to understand some basic internal workings |
| `follower_debug`     | `Float32MultiArray` | if `publish_debug_info` is enabled: `processing time`, `current_heading`, `lookahead_heading`, `heading_error`, `cross_track_error` and `target_velocity` are outputted |


#### Output to `vehicle_cmd`
* `ctrl_cmd/linear_velocity` - taken from the waypoints closest to `base_link` and interpolated between the waypoints
* `ctrl_cmd/linear_acceleration` - currently, constant 0.0 is used
* `ctrl_cmd/steering_angle` - calculated using pure_pursuit algorithm
* `lamp_cmd/l` and `lamp_cmd/r` - blinker commands for left and right blinker (1 - on, 0 - off)



## stanley_follower

ROS node that receives a local path (Lane message) and a vehicle status (current_pose and current_velocity) and calculates the desired vehicle steering angle and throttle/brake command using Stanley control law. The implementation is based on the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

#### Parameters

| Name | Type | Default Value | Description |
| --- | --- | --- | --- |
| `~cte_gain` | float | `0.3` | Gain for cross-track error |
| `/vehicle/wheel_base` | float | `2.789` | Distance between front and rear axles of the vehicle |
| `heading_angle_limit` | float | `90.0` | Maximum steering angle for the vehicle |
| `lateral_error_limit` | float | `2.0` | Maximum lateral error from the path |
| `blinker_lookahead_time`   | double | `3.0`   | Lookahead time for blinker state (multiplied with velocity)            |
| `~publish_debug_info` | bool | `False` | Whether or not to publish debug information |
| `~nearest_neighbor_search` | string | `"kd_tree"` | Algorithm for nearest neighbor search. Possible values are `"ball_tree"`, `"kd_tree"`, and `"brute"`.

#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/planning/local_path` | `autoware_msgs/Lane` | Local path received from the path planner |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the vehicle |

#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/vehicle_cmd` | `autoware_msgs/VehicleCmd` | Command for steering angle and throttle/brake |
| `~follower_markers` | `visualization_msgs/MarkerArray` | if `publish_debug_info` is enabled: debug markers for visualization |
| `~follower_debug` | `std_msgs/Float32MultiArray` | if `publish_debug_info` is enabled: `processing time`, `current_heading`, `track_heading`, `heading_error`, `cross_track_error` and `target_velocity` data is published |

#### Output to `vehicle_cmd`

* `ctrl_cmd/linear_velocity` - taken from the waypoints closest to `base_link` and interpolated between the waypoints
* `ctrl_cmd/linear_acceleration` - currently constant 0.0 is used
* `ctrl_cmd/steering_angle` - calculated using stanley algorithm
* `lamp_cmd/l` and `lamp_cmd/r` - blinker commands for left and right blinker (1 - on, 0 - off)
