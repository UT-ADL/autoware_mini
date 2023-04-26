# Planning - waypoints


## waypoint_loader


ROS node to load waypoints from a CSV file and publish them on a ROS topic. Waypoint file should have the following structure (header row):
```wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag```


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `waypoints_file` | `string` | - | The path to the CSV file containing the waypoints. |
| `output_frame` | `string` | `"map"` | The frame ID to use for the published waypoints. |
| `wp_left_width` | `float` | `1.4` | The left width of the waypoints. |
| `wp_right_width` | `float` | `1.4` | The right width of the waypoints. |


#### Subscribed Topics

None.


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/global_path` | `autoware_msgs/Lane` | The lane message containing the loaded waypoints. |



## waypoint_saver

This node saves the current position and velocity of a vehicle as waypoints in a csv file with the following format: wp_id x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag. It also publishes markers of the waypoints on the "path_markers" topic. A more detailed explanation of saved fields:

* `wp_id` - waypoint id automatically incremented
* `x`, `y`, `z` - coordinates from `current_pose` message
* `yaw` - Orientation from `current_pose` message and converted into yaw
* `velocity` - speed from `current_velocity` message
* `steering_flag` - used for blinker information. Inside the code there is also a remapping.
* Other fields are currently not used by `waypoint_saver`


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `interval` | `float` | `1.0` | Minimum distance between consecutive waypoints in meters  (m)|
| `waypoints_file` | `string` | `"/tmp/waypoints.csv"` | Name and path of the output file for the waypoints |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/localization/current_pose` | [`geometry_msgs/PoseStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | Current pose of the vehicle |
| `/localization/current_velocity` | [`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)  | Current velocity of the vehicle |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/path_markers` | `MarkerArray` | Markers of the saved waypoints | 
