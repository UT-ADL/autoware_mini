# Planning - Visualization


## global_path_visualizer

ROS node that visualizes the global path with markers in RViz. It displays path waypoints as colored arrows (based on turn signal), adds velocity labels, and triangulates the path for better visibility.


#### Parameters

No node-specific parameters.


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `global_path` | `autoware_mini/Path` | Global path waypoints to visualize. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `global_path_markers` | `visualization_msgs/MarkerArray` | Marker array visualizing the global path, including arrows showing waypoint poses, velocity labels, and triangulated path visualization. |


## local_path_visualizer

ROS node that visualizes the local path with markers in RViz. It displays the generated local path with triangulated visualization, adds velocity labels, shows stopping points, and publishes planner status to dashboard overlays.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `safety_box_width` | float | `2.7` | Width of the safety box used for visualization. |
| `stopped_speed_limit` | float | `1.0` | Speed threshold to determine if an object is stopped. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `local_path` | `autoware_mini/Path` | Local path waypoints to visualize. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `local_path_markers` | `visualization_msgs/MarkerArray` | Marker array visualizing the local path, including triangulated path visualization, velocity labels, and stopping points. |
| `/dashboard/planner_status` | `jsk_rviz_plugins/OverlayText` | Current status of the planner displayed as overlay text. |
| `/dashboard/log_message` | `autoware_mini/Log` | Log of recent planner status changes displayed as overlay text. |
