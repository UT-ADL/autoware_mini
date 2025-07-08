# Planning - visualization


## global_path_visualizer

This node visualizes the global path received on the 'global_path' topic.


#### Parameters

No parameters.


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/global_path` | `autoware_msgs/Lane` | The global path to be visualized. |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/global_path_markers` | `visualization_msgs/MarkerArray` | The markers that represent the visualized global path. |



## lanelet2_map_visualizer

This node visualizes the Lanelet2 map. It loads the map using Lanelet2 and creates a `MarkerArray` message which contains markers for different parts of the map. The markers are published on the topic `lanelet2_map_markers`. Additionally, it listens to the topic `/detection/traffic_light_status` for traffic light status messages and publishes stop line markers with different colors based on the status of the traffic lights on the topic `stop_line_markers`.


#### Parameters

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `~lanelet2_map_name` | string | "" | The path to the Lanelet2 map file |
| `/localization/coordinate_transformer` | string | "utm" | The type of coordinate transformer to use |
| `/localization/use_custom_origin` | bool | false | Whether to use a custom origin or not |
| `/localization/utm_origin_lat` | float | 0.0 | The latitude of the UTM origin |
| `/localization/utm_origin_lon` | float | 0.0 | The longitude of the UTM origin |


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/detection/traffic_light_status` | `autoware_msgs/TrafficLightResultArray` | The topic the node listens to for traffic light status messages. |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `lanelet2_map_markers` |  | The topic on which the node publishes the markers for different parts of the map. |
| `stop_line_markers` | `visualization_msgs/MarkerArray` | The topic on which the node publishes the stop line markers. |



## local_path_visualizer

A ROS node that visualizes the local path and other relevant information.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `car_safety_radius` | `float` | `1.3` | Safety radius of the car used for visualization |
| `current_pose_to_car_front` | `float` | `4.0` | Distance between the current pose and the car front |
| `braking_safety_distance` | `float` | `2.0` | Safety distance to use for braking visualization |
| `stopping_speed_limit` | `float` | `1.0` | Speed limit (m/s) for other vehicles to consider them as stopped |



#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/local_path` | `autoware_msgs/Lane` | The local path to visualize |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/local_path_markers` | `visualization_msgs/MarkerArray` | The marker array that contains the local path visualization and other relevant information |



## speed_obstacle_visualizer

This node subscribes to `/planning/local_path`, `/control/vehicle_cmd`, and `/localization/current_velocity` topics and publishes the current speed, target speed, closest object distance, and closest object speed. Published values are used in Rviz to display the data (dashboard).


#### Parameters

No parameters.


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/planning/local_path` | `autoware_msgs/Lane` | Local path published by the planning module |
| `/control/vehicle_cmd` | `autoware_msgs/VehicleCmd` | Vehicle command published by the control module |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity published by the localization module |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/current_speed` | `std_msgs/Float32` | Current vehicle speed in meters per second |
| `/target_speed` | `std_msgs/Float32` | Target speed in meters per second |
| `/closest_object_distance` | `std_msgs/Float32` | Closest obstacle distance in meters |
| `/closest_object_speed` | `std_msgs/Float32` | Closest obstacle speed in kilometers per hour |

