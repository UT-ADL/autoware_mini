# Simulation

This folder contains nodes related to simulation.

## bicycle_simulation

ROS node that implements a simple bicycle model for testing waypoint following controllers. It uses the formulation where the desired point is at the center of the rear axle. Velocity and steering angle changes are instantaneous.


#### Parameters

| Name | Type | Default Value | Description |
| ---- | ---- | ------------- | ----------- |
| `~publish_rate` | int | `50` | Rate in Hz at which to publish simulation data. |
| `wheel_base` | float | `2.789` | Distance between rear and front axle. |
| `acceleration_limit` | float | `1.0` | Maximum allowed acceleration (in m/s²). |
| `deceleration_limit` | float | `5.0` | Maximum allowed deceleration (in m/s²). |
| `/planning/default_acceleration` | float | `1.0` | Default acceleration when not specified (in m/s²). |
| `/planning/default_deceleration` | float | `1.0` | Default deceleration when not specified (in m/s²). |


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial location and orientation of the vehicle. Use the 2D Pose Estimate button in Rviz to set it. |
| `/initialvelocity` | `geometry_msgs/TwistStamped` | Initial velocity of the vehicle. |
| `/control/vehicle_cmd` | `autoware_mini/VehicleCommand` | Speed, steering angle, and turn signal are taken from the command and retained for publishing in `vehicle_status`. |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle according to the bicycle model. |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the vehicle based on the bicycle model. |
| `vehicle_status` | `autoware_mini/VehicleStatus` | Vehicle status including speed, steering angle, and turn signal state. |
| `bicycle_markers` | `visualization_msgs/MarkerArray` | Bicycle model visualization for debugging. |


## obstacle_simulation

ROS node that simulates obstacles by creating and removing detected objects in response to mouse clicks in the Rviz visualization window.


#### Parameters

| Name | Type | Default Value | Description |
| ---- | ---- | ------------- | ----------- |
| `~publish_rate` | int | `10` | Rate in Hz at which to publish detected objects. |


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/clicked_point` | `geometry_msgs/PointStamped` | Mouse click in the Rviz visualization window. Used to add a new obstacle at the clicked position or remove an existing obstacle if clicked on it. |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | List of simulated obstacles published at the specified rate. Each object includes position, dimensions, and a convex hull. |
