# Planning - waypoints


## waypoint_loader

ROS node to load waypoints from a CSV file and publish them on a ROS topic. Waypoint file should have the following structure (header row):
```
x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag
```


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `waypoints_file` | string | `-` | The path to the CSV file containing the waypoints. |
| `output_frame` | string | `"map"` | The frame ID to use for the published waypoints. |



#### Subscribed Topics

None.


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `global_path` | `autoware_mini/Path` | The path message containing the loaded waypoints. |



## waypoint_saver

This node saves the current position and velocity of a vehicle as waypoints in a csv file with the following format: x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag. It also publishes markers of the waypoints on the "path_markers" topic. A more detailed explanation of saved fields:

* `x`, `y`, `z` - coordinates from `current_pose` message
* `yaw` - Orientation from `current_pose` message and converted into degrees
* `velocity` - speed from `current_velocity` message
* `steering_flag` - used for turn signal information
* Other fields are currently set to 0


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `waypoints_file` | string | `-` | Name and path of the output file for the waypoints |
| `interval` | float | `1.0` | Minimum distance between consecutive waypoints in meters |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the vehicle |
| `/vehicle/vehicle_status` | `autoware_mini/VehicleStatus` | Vehicle status information including turn signals |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `path_markers` | `visualization_msgs/MarkerArray` | Markers of the saved waypoints |
