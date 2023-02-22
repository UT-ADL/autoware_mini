# Control

This folder contains controllers - nodes that do waypoint following. Source of the waypoints can be the waypoint file or outputs from the local or global planner.

## pure_pursuit_follower

The implementation is based from the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

##### Parameters

* `planning_time` - used to calculate lookahead point distance by multiplying with speed
* `min_lookahead_distance` - distance from base_link 
* `wheel_base`

##### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `path` | `autoware_msgs/LaneArray` | topic for waypoints |
| `current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | current location and orientation of the car in `map` frame |
| `current_velocity` | [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) | velocity of the car |

##### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `vehicle_cmd` | `autoware_msgs/VehicleCmd` | Main result from the follower is steering_angle, it publishes also linear_velocity and linear_acceleration, but these are not the outputs from pure_pursuit itself. It also reads blinker information from waypoints and adds that to command message. |
| `follower_markers` | `visualization_msgs/MarkerArray` | follower specific visualization topic that helps to understand some basic internal workings |

##### Output to `vehicle_cmd`
* `ctrl_cmd/linear_velocity` - taken from **lookahead** point
* `ctrl_cmd/linear_acceleration` - currently constant 0.0 is used
* `ctrl_cmd/steering_angle` - calculated using pure_pursuit algorithm
* `lamp_cmd/l` and `lamp_cmd/r` - blinker commands for left and right blinker (1 - on, 0 - off)

## stanley_follower

The implementation is based from the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

##### Parameters
* `wheel_base`

##### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `path` | `autoware_msgs/LaneArray` | topic for waypoints |
| `current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | current location and orientation of the car in `map` frame |
| `current_velocity` | [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) | velocity of the car |

##### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `vehicle_cmd` | `autoware_msgs/VehicleCmd` | Main result from the follower is steering_angle, it publishes also linear_velocity and linear_acceleration, but these are not the outputs from pure_pursuit itself.  It also reads blinker information from waypoints and adds that to command message. |
| `follower_markers` | `visualization_msgs/MarkerArray` | follower specific visualization topic that helps to understand some basic internal workings |

##### Output to `vehicle_cmd`
* `ctrl_cmd/linear_velocity` - taken from **nearest** point
* `ctrl_cmd/linear_acceleration` - currently constant 0.0 is used
* `ctrl_cmd/steering_angle` - calculated using stanley algorithm
* `lamp_cmd/l` and `lamp_cmd/r` - blinker commands for left and right blinker (1 - on, 0 - off)
