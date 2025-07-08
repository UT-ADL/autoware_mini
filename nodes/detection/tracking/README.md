# Detection - Tracking

## ema_tracker

ROS node that tracks detected objects using an Exponential Moving Average (EMA) approach. It associates incoming detections with existing tracks, estimates object velocities and accelerations, and manages track lifecycle.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- |---------------| ------------ |
| `~enable_initial_velocity_estimate` | bool | `False`       | Whether to make the initial velocity of an object equal to its first velocity estimate instead of zero. |
| `~enable_initial_acceleration_estimate` | bool | `False`       | Whether to make the initial acceleration of an object equal to its first acceleration estimate instead of zero. |
| `~enable_missed_detection_propagation` | bool | `True`        | Whether to propagate missed detections based on their last known velocity. |
| `~detection_counter_threshold` | int | `4`           | Number of detections required before an object is published as a track. |
| `~missed_counter_threshold` | int | `2`           | Number of consecutive missed detections before a track is deleted. |
| `~iou_threshold` | float | `0.1`         | Minimum IoU required to associate a detection with a track when using IoU-based association. |
| `~velocity_gain` | float | `0.2`         | Weight given to new velocity measurements in the EMA update. |
| `~acceleration_gain` | float | `0.0`         | Weight given to new acceleration measurements in the EMA update. |
| `~association_method` | string | `iou`         | Method used to associate detections with tracks. Can be `iou` or `euclidean`. |
| `~max_euclidean_distance` | float | `2.0`         | Maximum distance for associating a detection with a track when using Euclidean distance-based association. |
| `~update_heading_bboxes` | bool | `True`        | Whether to update object heading and dimensions based on velocity vector. |
| `/planning/stopped_speed_limit` | float | `1.0`         | Speed threshold to determine if an object is stopped. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Array of detected objects from the perception pipeline. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `tracked_objects` | `autoware_mini/DetectedObjectArray` | Array of tracked objects with consistent IDs, estimated velocities, and accelerations. Only objects that have been detected at least `detection_counter_threshold` times are published. |
