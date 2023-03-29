# Simulation

This folder contains nodes related to simulation.

## bicycle_simulation

Implements simple bicycle model for testing waypoint followers based on the blog post [Simple Understanding of Kinematic Bicycle Model](https://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html). It uses the formulation where desired point is at the center of rear axle. Velocity and steering angle changes are instantaneous.

#### Parameters

* `publish_rate` - frequency of publishing state of the vehicle
* `wheel_base` - distance between rear and front axle

#### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `vehicle_cmd` | [autoware_msgs/VehicleCmd](https://gitlab.com/astuff/autoware.ai/messages/-/blob/as/master/autoware_msgs/msg/VehicleCmd.msg) | Velocity is taken from `ctrl_cmd.linear_velocity` and steering angle from `ctrl_cmd.steering_angle`. Blinker state from `lamp_cmd.l` and `lamp_cmd.r` is retained for publishing in `vehicle_status`. |
| `/initialpose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | Initial location and orientation of the vehicle. Use 2D Pose Estimate button in Rviz to set it.|

#### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | Current pose of the vehicle according to the bicycle model. |
| `current_velocity` | [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) | Current velocity of the vehicle. The same as input command. |
| `bicycle_markers` | `visualization_msgs/MarkerArray` | Bicycle model visualization for debugging. |
| `vehicle_status` | [autoware_msgs/VehicleStatus](https://gitlab.com/astuff/autoware.ai/messages/-/blob/as/master/autoware_msgs/msg/VehicleStatus.msg) | Vehicle status for completeness sake. Only `speed`, `angle` and `lamp` fields are meaningfully populated. |