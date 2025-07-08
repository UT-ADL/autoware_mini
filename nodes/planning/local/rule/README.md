# Planning - Rule-based Planner

This directory contains rule-based planner that contribute to the local planner module in Autoware Mini. Each planner examines specific scenarios that can create collisions and publishes corresponding potential collision points that are then merged and used for trajectory planning.

![Rule-based Planner Architecture](/images/nodes/local_planning_rule.png)

## Local Path Extractor

The `local_path_extractor` node extracts a portion of the global path around the vehicle's current position to create a local path.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `local_path_length` | double | `100` | Maximum length of the local path (m) |
| `publish_rate` | int | `10` | Frequency to publish the local path (Hz) |
| `lookahead_distance` | double | `10` | Distance to look ahead on the path for better path following (m) |
| `distance_to_lookahead_path_limit` | double | `2.5` | Maximum allowed distance from lookahead path before recalculating (m) |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle |
| `global_path` | `autoware_mini/Path` | Global path to follow |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `extracted_local_path` | `autoware_mini/Path` | Extracted portion of the global path |

## Goal Stop Checker

The `goal_stop_checker` node ensures the vehicle stops at the goal point of the global path.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_goal` | double | `0.1` | Distance to stop before the goal point (m) |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `extracted_local_path` | `autoware_mini/Path` | Local path to check |
| `global_path` | `autoware_mini/Path` | Global path containing the goal |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `goal_collision_points` | `sensor_msgs/PointCloud2` | Collision points at the goal |

## Automatic Stop Checker

The `automatic_stop_checker` node detects stop lines on the path and ensures the vehicle stops at them until manually given permission to proceed.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `lanelet2_map_path` | string | `-` | Path of the lanelet2 map file to load |
| `braking_safety_distance_stop_line` | double | `0.1` | Distance to stop before the stop line (m) |
| `keep_stop_line_for` | double | `15.0` | Time in seconds to keep a stop line removed after pressing "Let's Go" button (s) |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `global_path` | `autoware_mini/Path` | Global path containing stop lines |
| `extracted_local_path` | `autoware_mini/Path` | Local path to check |
| `lets_go` | `std_msgs/Int32` | Command to proceed through a stop line |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `lets_go` | `std_msgs/Int32` | Command broadcast to proceed through a stop line |
| `stop_line_collision_points` | `sensor_msgs/PointCloud2` | Collision points at stop lines |
| `/dashboard/log_message` | `autoware_mini/Log` | Log messages for dashboard display |

### Services

| Name | Type | Description |
|------|------|-------------|
| `service_lets_go` | `std_srvs/Empty` | Service call to simulate Go button press from rviz |

## Object Collision Checker

The `object_collision_checker` node detects potential collisions with obstacles on the path.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `safety_box_width` | double | `2.7` | Width of the safety buffer around the path (m) |
| `stopped_speed_limit` | double | `1.0` | Speed below which an object is considered stopped (m/s) |
| `braking_safety_distance_obstacle` | double | `4.0` | Distance to stop before an obstacle (m) |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `extracted_local_path` | `autoware_mini/Path` | Local path to check |
| `/detection/tracked_objects` | `autoware_mini/DetectedObjectArray` | Detected objects around the vehicle |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `object_collision_points` | `sensor_msgs/PointCloud2` | Collision points for detected objects |

## Pedestrian Crosswalk Checker

The `pedestrian_crosswalk_checker` node handles crosswalks and checks if pedestrians are or will be crossing them.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `lanelet2_map_path` | string | `-` | Name of the lanelet2 map to load |
| `safety_box_width` | double | `2.7` | Width of the safety buffer around the path (m) |
| `stopped_speed_limit` | double | `1.0` | Speed below which an object is considered stopped (m/s) |
| `braking_safety_distance_crosswalk` | double | `6.0` | Distance to stop before a crosswalk (m) |
| `crossing_angle_max_limit` | double | `60.0` | Maximum angle between object trajectory and path to be considered crossing (degrees) |
| `ignore_static_obstacles` | bool | `True` | Whether to ignore stationary objects |
| `crosswalk_maximum_deceleration` | double | `3.0` | Maximum deceleration allowed for crosswalk stops (m/s²) |
| `prediction_counter_min_limit` | int | `4` | Minimum number of consecutive predictions to trigger a stop |
| `use_object_width` | bool | `True` | Whether to use object width in collision checks |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `global_path` | `autoware_mini/Path` | Global path containing crosswalks |
| `extracted_local_path` | `autoware_mini/Path` | Local path to check |
| `/detection/predicted_objects` | `autoware_mini/DetectedObjectArray` | Predicted objects with trajectories |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `crosswalk_collision_points` | `sensor_msgs/PointCloud2` | Collision points at crosswalks |

## Traffic Light Stopline Checker

The `traffic_light_stopline_checker` node monitors traffic light states and creates collision points at red traffic lights.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `lanelet2_map_path` | string | `-` | Name of the lanelet2 map to load |
| `tfl_force_stop_speed_limit` | double | `5.0` | Speed below which vehicle must stop at a red light (km/h) |
| `braking_safety_distance_stopline` | double | `2` | Distance to stop before the stop line (m) |
| `tfl_maximum_deceleration` | double | `2.8` | Maximum deceleration for traffic light stops (m/s²) |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `extracted_local_path` | `autoware_mini/Path` | Local path to check |
| `/detection/traffic_light_status` | `autoware_mini/TrafficLightResultArray` | Status of traffic lights |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the vehicle |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `tfl_stopline_collision_points` | `sensor_msgs/PointCloud2` | Collision points at traffic light stop lines |

## Trajectory Collision Checker

The `trajectory_collision_checker` node checks for potential collisions with other vehicles' predicted trajectories.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `lanelet2_map_path` | string | `-` | Name of the lanelet2 map to load |
| `safety_box_width` | double | `2.7` | Width of the safety buffer around the path (m) |
| `safety_box_length` | double | `5.0` | Length of the safety buffer around the vehicle (m) |
| `braking_safety_distance_trajectory` | double | `4` | Distance to maintain before potential collision point (m) |
| `heading_alignment_limit` | double | `30.0` | Maximum angle between object heading and path to be considered aligned (degrees) |
| `safety_time_ego_front` | double | `2.0` | Safety time buffer for front of ego vehicle (s) |
| `safety_time_ego_rear` | double | `1.0` | Safety time buffer for rear of ego vehicle (s) |
| `use_object_width` | bool | `True` | Whether to use object width in collision checks |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `/detection/predicted_objects_map` | `autoware_mini/DetectedObjectArray` | Predicted objects with trajectories |
| `extracted_local_path` | `autoware_mini/Path` | Local path to check |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the vehicle |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `trajectory_collision_points` | `sensor_msgs/PointCloud2` | Collision points from trajectory intersections |

## Yielding Checker

The `yielding_checker` node handles yielding situations at yield signs or intersections.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `lanelet2_map_path` | string | `-` | Name of the lanelet2 map to load |
| `safety_box_width` | double | `2.7` | Width of the safety buffer around the path (m) |
| `braking_safety_distance_yield` | double | `1.0` | Distance to stop before a yield point (m) |
| `yielding_maximum_deceleration` | double | `2.8` | Maximum deceleration for yielding (m/s²) |
| `yielding_distance_limit` | double | `40.0` | Maximum distance to consider for yielding after a yield line (m) |
| `heading_alignment_limit` | double | `30.0` | Maximum angle between object heading and path to be considered aligned (degrees) |
| `use_object_width` | bool | `True` | Whether to use object width in collision checks |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `/detection/predicted_objects_map` | `autoware_mini/DetectedObjectArray` | Predicted objects with trajectories |
| `global_path` | `autoware_mini/Path` | Global path containing yield points |
| `extracted_local_path` | `autoware_mini/Path` | Local path to check |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `yielding_collision_points` | `sensor_msgs/PointCloud2` | Collision points at yield locations |

## Speed Planner

The `speed_planner` node generates the final trajectory by adjusting speeds based on all collision points.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `synchronization_method` | string | `-` | Method for synchronizing input messages ("approximate" or "exact") |
| `default_deceleration` | double | `1.0` | Default deceleration to use for speed planning (m/s²) |
| `braking_reaction_time` | double | `1.6` | Reaction time to account for in braking calculations (s) |
| `synchronization_queue_size` | int | `4` | Queue size for message synchronization |
| `synchronization_slop` | double | `0.15` | Time tolerance for message synchronization (seconds) |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the vehicle |
| `collision_points` | `sensor_msgs/PointCloud2` | Merged collision points from all checkers |
| `extracted_local_path` | `autoware_mini/Path` | Local path to plan speeds for |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `local_path` | `autoware_mini/Path` | Final path with planned speeds |
| `/dashboard/log_message` | `autoware_mini/Log` | Log messages for dashboard display |

## Collision Points Merger

The `collision_points_merger` node merges all collision points from various checkers into a single point cloud.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `synchronization_method` | string | `-` | Synchronization method for input messages ("approximate" or "exact") |
| `enable_goal_checker` | bool | `-` | Whether to enable goal checker |
| `enable_object_checker` | bool | `-` | Whether to enable object checker |
| `enable_auto_stop_checker` | bool | `-` | Whether to enable automatic stop checker |
| `enable_traffic_light_checker` | bool | `-` | Whether to enable traffic light checker |
| `enable_crosswalk_checker` | bool | `-` | Whether to enable crosswalk checker |
| `enable_yielding_checker` | bool | `-` | Whether to enable yielding checker |
| `enable_trajectory_checker` | bool | `-` | Whether to enable trajectory checker |
| `synchronization_queue_size` | int | `4` | Queue size for message synchronization |
| `synchronization_slop` | double | `0.15` | Time tolerance for message synchronization (seconds) |

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `goal_collision_points` | `sensor_msgs/PointCloud2` | Collision points from goal checker |
| `object_collision_points` | `sensor_msgs/PointCloud2` | Collision points from object checker |
| `stop_line_collision_points` | `sensor_msgs/PointCloud2` | Collision points from stop line checker |
| `tfl_stopline_collision_points` | `sensor_msgs/PointCloud2` | Collision points from traffic light checker |
| `crosswalk_collision_points` | `sensor_msgs/PointCloud2` | Collision points from crosswalk checker |
| `yielding_collision_points` | `sensor_msgs/PointCloud2` | Collision points from yielding checker |
| `trajectory_collision_points` | `sensor_msgs/PointCloud2` | Collision points from trajectory checker |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `collision_points` | `sensor_msgs/PointCloud2` | Merged collision points |

