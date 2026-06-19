# Planning - Openpilot

## openpilot_local_planner

ROS node that processes predictions from Openpilot and generates a local path with proper velocities and turn signal states. The node transforms the Openpilot trajectory prediction to the desired output frame and combines it with velocity data.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `transform_timeout` | float | `0.06` | Timeout in seconds for waiting for a transform to become available. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current vehicle pose. |
| `global_path` | `autoware_mini/Path` | Global path to follow. |
| `/openpilot/position` | `vehicle_platform/Float32MultiArrayStamped` | Position predictions from Openpilot. |
| `/openpilot/velocity` | `vehicle_platform/Float32MultiArrayStamped` | Velocity predictions from Openpilot. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `openpilot_local_path` | `autoware_mini/Path` | Local path generated from Openpilot predictions. Contains waypoints with position, heading, velocity, and turn signal information. |
