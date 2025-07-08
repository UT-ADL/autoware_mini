# Platform - bag


## pause_bag

ROS node that interfaces with the CARLA simulator control commands to pause and resume bag playback.


#### Parameters

No configurable parameters found in configuration files.


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/carla/control` | `carla_msgs/CarlaControl` | Receives control commands for CARLA simulation (PLAY, PAUSE, STEP_ONCE). |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/carla/status` | `carla_msgs/CarlaStatus` | Publishes the current status of the CARLA simulation. |


#### Services

No services provided by this node.


## record_bag

ROS node for recording bag files with configurable topic blacklisting.


#### Parameters

| Name | Type | Default | Description |
| ----- | ----- | ------------ | ------------ |
| `~blacklist_file` | string | - | Path to the file containing blacklisted topics. |
| `~recorded_bags_dir` | string | - | Directory where recorded bag files will be saved. |
| `~bag_name` | string | - | Default name for recorded bag files. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/record_command` | `jsk_rviz_plugins/RecordCommand` | Receives commands to start/stop recording. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/dashboard/recording_symbol` | `jsk_rviz_plugins/OverlayText` | Displays recording status indicator. |
| `/dashboard/log_message` | `autoware_mini/Log` | Publishes log messages related to recording. |
| `/record_command` | `jsk_rviz_plugins/RecordCommand` | Publishes record commands. |


#### Services

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/dashboard/start_record` | `std_srvs/Empty` | Service to start recording a bag file. |
| `/dashboard/stop_record` | `std_srvs/Empty` | Service to stop recording a bag file. |
