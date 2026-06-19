# Perception - Filtering

## detection_range_filter

ROS node that filters out detected objects that are beyond a specified range from the car's front.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- |---------------| ------------ |
| `detection_range` | float | -             | Maximum distance from car front for objects to be kept |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/detected_objects` | `autoware_mini/DetectedObjectArray` | The input detected objects |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/detected_objects_filtered` | `autoware_mini/DetectedObjectArray` | Filtered detected objects within range |


## road_area_filter

ROS node that filters detected objects based on whether they're within a road area defined in a GeoJSON file.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- |---------------| ------------ |
| `road_area_file_path` | string | -             | Path to the GeoJSON file containing road area information |
| `filtering_method` | string | `"centroid"`  | Method to filter objects. Options: 'centroid' (centroid inside road area), 'intersects' (has overlap with road area), or 'within' (object completely inside road area) |
| `use_map_extraction` | bool | `True`        | Whether to extract a smaller map area around the ego vehicle for visualization |
| `map_extraction_distance` | float | `150`         | Distance in meters from ego vehicle to extract map data |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the vehicle |
| `/detected_objects` | `autoware_mini/DetectedObjectArray` | The input detected objects |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/detected_objects_filtered` | `autoware_mini/DetectedObjectArray` | Filtered detected objects within road area |
| `/road_area_markers` | `visualization_msgs/MarkerArray` | Markers for visualizing the road area |
