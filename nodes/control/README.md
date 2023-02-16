# Control

This folder contains controllers - nodes that do waypoint following. Source of the waypoints can be the waypoint file or outputs from the local or global planner.

## pure_pursuit_follower

The implementation is based from the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

##### Parameters

* `planning_time` - default: 2 s - used to calculate lookahead point distance by multiplying with speed
* `min_lookahead_distance` - default: 5.5 m - distance from base_link 
* `wheel_base` - default 2.789 m

##### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `/path` | `autoware_msgs/Lane` | topic for waypoints |
| `/current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | current location and orientation of the car in `map` frame |
| `/current_velocity` | [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) | velocity of the car |

##### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `/vehicle_cmd` | `autoware_msgs/VehicleCmd` | Main result from the follower is steering_angle, it publishes also linear_velocity and linear_acceleration, but these are not the outputs from pure_pursuit itself. It also reads blinker information from waypoints and adds that to command message. |
| `/follower_markers` | `visualization_msgs/MarkerArray` | follower specific visualization topic that helps to understand some basic internal workings |
| `/follower_debugging` | TODO! | TODO: lateral error, compute time? |


## stanley_follower

The implementation is based from the blog post [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://www.shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html).

##### Parameters
* `wheel_base` - default 2.789 m

##### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `/path` | `autoware_msgs/Lane` | topic for waypoints |
| `/current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | current location and orientation of the car in `map` frame |
| `/current_velocity` | [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) | velocity of the car |

##### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `/vehicle_cmd` | `autoware_msgs/VehicleCmd` | Main result from the follower is steering_angle, it publishes also linear_velocity and linear_acceleration, but these are not the outputs from pure_pursuit itself.  It also reads blinker information from waypoints and adds that to command message. |
| `/follower_markers` | `visualization_msgs/MarkerArray` | follower specific visualization topic that helps to understand some basic internal workings |
| `/follower_debugging` | TODO! | TODO: lateral error, compute time? |
