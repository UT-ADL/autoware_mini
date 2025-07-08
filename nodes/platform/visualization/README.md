# Visualization

This section contains nodes for visualizing vehicle state and status information.

## carla_status_visualizer

ROS node that converts Autoware vehicle status messages to CARLA status format for visualization in CARLA.


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/vehicle/vehicle_status` | `autoware_mini/VehicleStatus` | Vehicle status information from Autoware. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/carla/ego_vehicle/vehicle_status` | `carla_msgs/CarlaEgoVehicleStatus` | Vehicle status in CARLA format. |


## vehicle_state_visualizer

ROS node for visualizing vehicle state information in RViz using overlay texts and images.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~image_path` | string | `-` | Path to the directory containing steering wheel images. |
| `/vehicle/steer_ratio` | float | `16.135` | Steering ratio for converting wheel angle to steering wheel rotation. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/vehicle/vehicle_status` | `autoware_mini/VehicleStatus` | Vehicle status information. |
| `/control/vehicle_cmd` | `autoware_mini/VehicleCmd` | Vehicle command information, including turn signals. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `vehicle_drivemode` | `jsk_rviz_plugins/OverlayText` | Overlay text displaying the current drive mode (AUTONOMOUS/MANUAL). |
| `steering_wheel` | `sensor_msgs/Image` | Rotated image of a steering wheel showing the current steering angle. |
| `right_blinker_cmd` | `jsk_rviz_plugins/OverlayText` | Overlay text showing right turn signal command. |
| `right_blinker_arrow` | `jsk_rviz_plugins/OverlayText` | Overlay text showing right turn signal status. |
| `left_blinker_cmd` | `jsk_rviz_plugins/OverlayText` | Overlay text showing left turn signal command. |
| `left_blinker_arrow` | `jsk_rviz_plugins/OverlayText` | Overlay text showing left turn signal status. |
