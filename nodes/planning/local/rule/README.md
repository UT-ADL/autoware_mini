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

## Collision Checker

The `collision_checker` node runs up to 13 rule-based safety checkers against the local path and publishes merged collision points for the speed planner. Checkers can run sequentially or in parallel via a thread pool. Each checker is independently enabled via launch parameters.

### Subscribed Topics

| Name | Type | Description |
|------|------|-------------|
| `extracted_local_path` | `autoware_mini/Path` | Local path to check (triggers all checkers) |
| `global_path` | `autoware_mini/Path` | Global path containing the goal (goal checker) |
| `/perception/predicted_objects_map` | `autoware_mini/DetectedObjectArray` | Objects with map-based trajectory predictions |
| `/perception/predicted_objects` | `autoware_mini/DetectedObjectArray` | Objects with raw trajectory predictions (crosswalk checker) |
| `/perception/tracked_objects` | `autoware_mini/DetectedObjectArray` | Objects with bounding boxes only (object checker) |
| `/perception/traffic_light_status` | `autoware_mini/StopLineStatusArray` | Traffic light stop line statuses |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the vehicle |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle |
| `confirm_drive` | `std_msgs/Int32` | Command to proceed through a manual yield stop line |

### Published Topics

| Name | Type | Description |
|------|------|-------------|
| `collision_points` | `sensor_msgs/PointCloud2` | Merged collision points from all enabled checkers |
| `stop_line_status` | `autoware_mini/StopLineStatusArray` | Stop line statuses for visualization |
| `confirm_drive` | `std_msgs/Int32` | Command broadcast to proceed through a stop line (manual yield checker) |
| `/dashboard/log_message` | `autoware_mini/Log` | Log messages for dashboard display (manual yield checker) |

### Services

| Name | Type | Description |
|------|------|-------------|
| `service_confirm_drive` | `std_srvs/Empty` | Service call to simulate Confirm Drive button press from rviz |

### Global Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `safety_box_width` | double | `2.7` | Width of the narrow safety buffer around the path (m) |
| `wide_safety_box_width` | double | `3.1` | Width of the wide safety buffer for pre-filtering (m) |
| `safety_box_length` | double | `5.0` | Length of the safety buffer around the vehicle (m) |
| `stopped_speed_limit` | double | `1.0` | Speed below which an object is considered stopped (m/s) |
| `ego_vehicle_stopped_speed_limit` | double | `0.1` | Speed below which the ego vehicle is considered stopped (m/s) |
| `from_behind_heading_limit` | double | `30` | Max heading difference to consider trajectory coming from behind (degrees) |
| `collision_point_interval` | double | `1.0` | Max segment length for densifying collision points (m) |
| `parallel_paths` | int | `9` | Number of parallel paths for swerving |
| `parallel_paths_interval` | double | `0.3` | Interval between parallel paths (m) |

### Node-local Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `lanelet2_map_path` | string | `-` | Path of the lanelet2 map file to load |
| `parallel_checkers` | bool | `false` | Run checkers in parallel using a thread pool |
| `enable_goal_checker` | bool | `-` | Enable goal stop checker |
| `enable_object_checker` | bool | `-` | Enable object collision checker |
| `enable_traffic_light_checker` | bool | `-` | Enable traffic light stop line checker |
| `enable_crosswalk_checker` | bool | `-` | Enable pedestrian crosswalk checker |
| `enable_yielding_checker` | bool | `-` | Enable yielding checker |
| `enable_give_way_checker` | bool | `-` | Enable give way checker |
| `enable_bus_stop_checker` | bool | `-` | Enable bus stop checker |
| `enable_trajectory_checker` | bool | `-` | Enable trajectory collision checker |
| `enable_right_of_way_checker` | bool | `-` | Enable right of way checker |
| `enable_manual_yield_checker` | bool | `-` | Enable manual yield checker |
| `enable_stop_sign_checker` | bool | `-` | Enable stop sign checker |
| `enable_lane_boundary_checker` | bool | `-` | Enable lane boundary checker |

### Checker Parameters

#### goal_stop_checker

Stops the vehicle at the goal point of the global path.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_goal` | double | `0.1` | Distance to stop before the goal point (m) |

#### object_collision_checker

Detects potential collisions with tracked obstacles on the path. Classifies objects as moving or stationary based on their velocity projected onto the path heading.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `approaching_deceleration` | double | `3.5` | Deceleration applied when object approaches ego (m/s2) |
| `braking_safety_distance_obstacle` | double | `4` | Distance to stop before an obstacle (m) |

#### traffic_light_stop_line_checker

Creates collision points at red traffic light stop lines. Forces a hard stop when ego speed is below the force stop limit.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_stop_line` | double | `2` | Distance to stop before the stop line (m) |
| `tfl_force_stop_speed_limit` | double | `5.0` | Speed below which vehicle must hard-stop at a red light (km/h) |
| `tfl_deceleration_limit` | double | `3.0` | Maximum deceleration for traffic light stops (m/s2) |

#### pedestrian_crosswalk_checker

Handles crosswalks by checking if objects are on the crosswalk or have predicted trajectories through it. Uses approach angle to distinguish crossing from parallel movement.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_crosswalk` | double | `4` | Distance to stop before a crosswalk (m) |
| `crossing_angle_max_limit` | double | `60` | Maximum angle to be considered approaching or departing the path (degrees) |
| `ignore_static_obstacles` | bool | `True` | Whether to ignore objects without predicted trajectories |
| `crosswalk_deceleration_limit` | double | `3.0` | Maximum deceleration for crosswalk stops (m/s2) |
| `prediction_counter_min_limit` | int | `4` | Minimum consecutive predictions before triggering a stop |

#### yielding_checker

Stops at the first yield stop line when a predicted trajectory intersects the path beyond the stop line within a distance limit.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_yield` | double | `1.0` | Distance to stop before a yield point (m) |
| `yielding_distance_limit` | double | `40.0` | Maximum distance beyond yield line to check for crossing traffic (m) |
| `yielding_deceleration_limit` | double | `3.0` | Maximum deceleration for yielding (m/s2) |

#### give_way_checker

Handles give way at unregulated intersections. Checks if objects approach from the right when going straight/turning right, or from any direction when turning left.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_give_way` | double | `4.0` | Distance to stop before a give way point (m) |
| `give_way_deceleration_limit` | double | `3.0` | Maximum deceleration for give way situations (m/s2) |
| `give_way_counter_min_limit` | int | `3` | Minimum consecutive detections to trigger give way |
| `check_right_turn` | bool | `True` | Whether to check give way when turning right |
| `check_left_turn` | bool | `True` | Whether to check give way when turning left |

#### right_of_way_checker

Checks if predicted trajectories or objects intersect right-of-way regulatory element polygons from the Lanelet2 map, with heading alignment verification.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_right_of_way` | double | `1.0` | Distance to stop before a yield point (m) |
| `right_of_way_deceleration_limit` | double | `3.0` | Maximum deceleration for right of way (m/s2) |
| `heading_alignment_limit` | double | `30.0` | Max heading difference to consider object aligned with lanelet (degrees) |
| `right_turn_check_range` | double | `15.0` | Distance range to check for right turn signal (m) |

#### trajectory_collision_checker

Checks for time-based collisions with predicted trajectories. When ego is stopped, uses distance-only check. When moving, computes arrival/leaving time intervals for both ego and object at each intersection point.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `trajectory_collision_deceleration` | double | `3.0` | Deceleration applied for trajectory collisions (m/s2) |
| `braking_safety_distance_trajectory` | double | `4.0` | Distance to stop before collision point (m) |
| `safety_time_ego_front` | double | `2.0` | Safety time buffer subtracted from ego arrival time (s) |
| `safety_time_ego_rear` | double | `1.0` | Safety time buffer added to ego leaving time (s) |
| `trajectory_counter_min_limit` | int | `5` | Minimum consecutive detections before triggering |
| `no_time_collision_check_distance` | double | `10.0` | Distance threshold for stopped ego collision check (m) |

#### manual_yield_checker

Forces stop at manual yield stop lines until a human confirms via `confirm_drive` topic or service. The override expires after a timeout or when the stop line is passed.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_stop_line` | double | `0.95` | Distance to stop before the stop line (m) |
| `keep_stop_line_for` | double | `15.0` | Duration to keep stop line overridden after confirmation (s) |

#### stop_sign_checker

Ensures the vehicle comes to a complete stop at stop signs. Once stopped within range, the stop requirement is fulfilled and the vehicle proceeds.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_stop_sign` | double | `1.0` | Distance to stop before a stop sign (m) |
| `stopping_distance` | double | `2.0` | Distance within which stop sign stopping is disabled (m) |

#### lane_boundary_checker

Adds collision points at lane boundary positions to keep the vehicle within lane. Allows crossing dashed boundaries but blocks solid ones.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_lane_boundary` | double | `1.0` | Distance to stop before a lane boundary (m) |
| `min_collision_point_distance` | double | `3.0` | Minimum distance from ego to add boundary collision points (m) |

#### bus_stop_checker

Gives way to buses whose predicted trajectories intersect the path.

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `braking_safety_distance_bus_stop` | double | `0.0` | Distance to stop before a bus (m) |
| `bus_stop_deceleration_limit` | double | `3.0` | Maximum deceleration for bus stop situations (m/s2) |

## Speed Planner

The `speed_planner` node generates the final trajectory by adjusting speeds based on all collision points.

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `synchronization_method` | string | `-` | Method for synchronizing input messages ("approximate" or "exact") |
| `default_deceleration` | double | `1.0` | Default deceleration to use for speed planning (m/s2) |
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

