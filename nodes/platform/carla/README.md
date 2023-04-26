# Platform - Carla

## carla_detector

ROS node for converting ground truth detections from the CARLA simulator and publishing them as `autoware_msgs::DetectedObjectArray` messages.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `~use_offset` | `bool` | `True` | If true, will use the `localization.SimulationToUTMTransformer` class to convert the detected objects to the UTM coordinate system. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/carla/ground_truth_objects` | `derived_object_msgs::ObjectArray` | Subscribes to ground truth object detections from the CARLA simulator. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `detected_objects` | `autoware_msgs::DetectedObjectArray` | Publishes converted object detections as a `DetectedObjectArray` message. |



## carla_localizer

ROS node for ground truth localization.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `~use_offset` | `bool` | `True` | Whether to use an offset transformation from simulation coordinates to UTM coordinates. |
| `/localization/use_custom_origin` | `bool` | `True` | Whether to use a custom origin for UTM coordinates. |
| `/localization/utm_origin_lat` | `float` | | Latitude of the custom origin for UTM coordinates. |
| `/localization/utm_origin_lon` | `float` | | Longitude of the custom origin for UTM coordinates. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/carla/odometry` | `nav_msgs/Odometry` | Odometry message containing the current pose and velocity of the ego vehicle in the simulation coordinates.


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `current_pose` | `geometry_msgs/PoseStamped` | Current pose of the ego vehicle in the map frame. |
| `current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the ego vehicle in the ego vehicle frame. |
| `odometry` | `nav_msgs/Odometry` | Odometry message containing the current pose and velocity of the ego vehicle. |



## carla_novatel_driver

Converts carla GNSS and odometry to Novatel OEM7 messages.

#### Parameters

No parameters.


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GNSS fix |
| `/gps/fix_forward` | `sensor_msgs/NavSatFix` | Forward GNSS fix |
| `/carla/odometry` | `nav_msgs/Odometry` | Carla odometry |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/novatel/oem7/inspva` | `novatel_oem7_msgs/INSPVA` | Inertial Navigation System Solution message |
| `/novatel/oem7/bestpos` | `novatel_oem7_msgs/BESTPOS` | Best Position message |



## carla_traffic_light_detector

ROS node that detects traffic lights from the Carla simulator and publishes the detection result to a topic in the Autoware-compatible format.


#### Parameters

| Name | Type | Default value | Description |
| ---- | ---- | ------------- | ----------- |
| `~use_offset` | `bool` | `true` | Whether to use the UTM offset when converting the traffic light position from the simulator coordinates to the UTM coordinates. |
| `/localization/coordinate_transformer` | `string` | | The coordinate transformer used to load the map. Only "utm" is currently supported. |
| `/localization/use_custom_origin` | `bool` | `true` | Whether to use a custom origin. |
| `/localization/utm_origin_lat` | `float` | | The latitude of the UTM origin. |
| `/localization/utm_origin_lon` | `float` | | The longitude of the UTM origin. |
| `~lanelet2_map_name` | `string` | | The name of the Lanelet2 map file to load. |


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/carla/traffic_lights/info` | `carla_msgs/CarlaTrafficLightInfoList` | The Carla traffic light info topic. |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `traffic_light_status` | `autoware_msgs/TrafficLightResultArray` | The Autoware traffic light status topic. |



## carla_vehicle_interface

This node receives Autoware messages and Carla messages, and publishes AckermannDrive, VehicleStatus, and Float64 messages.


#### Parameters

| Name             | Type   | Default  | Description                                   |
| ---------------- | ------ | -------- | --------------------------------------------- |
| `~max_steer_angle`| `double` | `1.2217` | Maximum steering angle in radians (default: 70) |


#### Subscribed Topics

| Name                     | Type                              | Description                                    |
| ------------------------| ----------------------------------| -----------------------------------------------|
| `/control/vehicle_cmd`   | `autoware_msgs/VehicleCmd`        | The control message containing vehicle commands |
| `/carla/ego_vehicle/vehicle_info` | `carla_msgs/CarlaEgoVehicleInfo` | The information message containing vehicle information |
| `/carla/ego_vehicle/vehicle_status` | `carla_msgs/CarlaEgoVehicleStatus` | The status message containing vehicle status |


#### Published Topics

| Name                             | Type                              | Description                        |
| -------------------------------- | ----------------------------------| -----------------------------------|
| `/carla/ego_vehicle/ackermann_cmd`| `ackermann_msgs/AckermannDrive`   | The ackermann drive command message |
| `/carla/ego_vehicle/target_speed`| `std_msgs/Float64`                | The target speed message (currently used only by scenario runner) |
| `/vehicle/vehicle_status`        | `autoware_msgs/VehicleStatus`     | The vehicle status message         |



## carla_waypoints_publisher

Receive a path from carla_ros_waypoint_publisher and convert it to autoware format.


#### Parameters

No parameters.


#### Subscribed Topics

| Name                             | Type              | Description                         |
| -------------------------------- | ----------------- | ----------------------------------- |
| `/carla/ego_vehicle/waypoints`     | `nav_msgs/Path`     | The path from carla_ros_waypoint_publisher |


#### Published Topics

| Name                  | Type               | Description                    |
| --------------------- | ------------------ | ------------------------------ |
| `/global_path`          | `autoware_msgs/LaneArray` | The converted path in Autoware format |
