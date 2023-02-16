# Tools

## waypoint_saver

Records waypoints, speed, yaw and blinker information and writes to csv file.

##### Parameters

* `interval` - distance between waypoints in meters (default: 1.0 m).
* `file_name` - output file name where to save waypoints (default: /tmp/waypoints.csv).

##### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `/current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | waypoint coordinates and yaw |
| `/current_velocity` | [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) | velocity |
| `/pacmod/parsed_tx/turn_rpt` | `pacmod_msgs.msg/SystemRptInt` | blinker information |

##### Output

Will record waypoint file with the following columns:

```wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag```

* `wp_id` - waypoint id automatically incremented
* `x`, `y`, `z` - coordinates from `/current_pose` message
* `yaw` - Orientation from `/current_pose` converted into heading angle
* `velocity` - speed from `/current_velocity`
* `steering_flag` - used for blinker information. Inside the code there is also a remapping.
* Other fields are currently not used by `waypoint_saver`

## waypoint_loader

* Loads waypoints from the waypoint file - csv file with the following columns

```wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag```

* Has no subscribers

##### Parameters
* `waypoints_file` - input waypoints file (full path)
* `output_frame` - default: `map`
* `publish_markers` - will publish waypoint markers to Rviz 

##### Publishes
| Topic | Type | Comment |
| --- | --- | --- |
| `/path` | autoware_msgs/Lane | Array of waypoints `autoware_msgs/Waypoint` |
| `/waypoint_markers` | visualization_msgs/MarkerArray | waypoint pose, velocity labels and path |
