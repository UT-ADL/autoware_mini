# Platform - Carla

## carla_detector

ROS node for converting ground truth detections from the CARLA simulator and publishing them as `autoware_mini::DetectedObjectArray` messages.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `~output_frame` | `string` | - | The output coordinate frame for detections. |


#### Subscribed Topics

| Name | Type                              | Description |
| --- |-----------------------------------| --- |
| `/carla/ego_vehicle/objects` | `derived_object_msgs/ObjectArray` | Subscribes to ground truth object detections from the CARLA simulator. |


#### Published Topics

| Name | Type                                | Description |
| --- |-------------------------------------| --- |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Publishes converted object detections as a `DetectedObjectArray` message. |



## carla_initialpose

ROS node for converting initial pose to simulation pose.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `/localization/utm_origin_lat` | `float` | `0.0` | Latitude of the custom origin for UTM coordinates. |
| `/localization/utm_origin_lon` | `float` | `0.0` | Longitude of the custom origin for UTM coordinates. |
| `/localization/use_custom_origin` | `bool` | `True` | Whether to use a custom origin for UTM coordinates. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | Current vehicle odometry to get z-coordinate information. |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial pose set by the user in RViz. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/carla/ego_vehicle/control/set_transform` | `geometry_msgs/Pose` | Publishes the pose to teleport the vehicle in CARLA simulation. |



## carla_localizer

ROS node for ground truth localization.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `/localization/utm_origin_lat` | `float` | `0.0` | Latitude of the custom origin for UTM coordinates. |
| `/localization/utm_origin_lon` | `float` | `0.0` | Longitude of the custom origin for UTM coordinates. |
| `/localization/use_custom_origin` | `bool` | `True` | Whether to use a custom origin for UTM coordinates. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | Odometry message containing the current pose and velocity of the ego vehicle in the simulation coordinates.


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `current_pose` | `geometry_msgs/PoseStamped` | Current pose of the ego vehicle in the map frame. |
| `current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the ego vehicle in the ego vehicle frame. |
| `odometry` | `nav_msgs/Odometry` | Odometry message containing the current pose and velocity of the ego vehicle. |



## carla_minimal_agent

ROS node that implements a minimal agent for the CARLA scenario runner. Publishes route waypoints and goal points.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `/localization/utm_origin_lat` | `float` | `0.0` | Latitude of the custom origin for UTM coordinates. |
| `/localization/utm_origin_lon` | `float` | `0.0` | Longitude of the custom origin for UTM coordinates. |
| `~init_goal_delay` | `int` | `5` | Delay in seconds before publishing the initial goal. |
| `~downsampling_interval` | `int` | `42` | Interval for downsampling route points. |
| `/localization/use_custom_origin` | `bool` | `True` | Whether to use a custom origin for UTM coordinates. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/carla/ego_vehicle/waypoints` | `nav_msgs/Path` | Publishes the route waypoints as a path. |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Publishes the goal points. |



## carla_novatel_driver

Converts carla GNSS and odometry to Novatel OEM7 messages.

#### Parameters

No parameters.


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GNSS fix |
| `/gps/fix_forward` | `sensor_msgs/NavSatFix` | Forward GNSS fix |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | Carla odometry |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/novatel/oem7/inspva` | `novatel_oem7_msgs/INSPVA` | Inertial Navigation System Solution message |
| `/novatel/oem7/bestpos` | `novatel_oem7_msgs/BESTPOS` | Best Position message |



## carla_route_saver

ROS node for saving vehicle route to XML file for later use in CARLA.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `~routes_file` | `string` | - | Path to save the routes XML file. |
| `~interval` | `int` | `2` | Distance interval between waypoints in meters. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/carla/odometry` | `nav_msgs/Odometry` | Odometry message containing the current pose of the ego vehicle. |



## carla_scenario_publisher

ROS node for publishing available CARLA scenarios from a specified folder.


#### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `~scenario_path` | `string` | - | Path to the folder containing scenario files (.xosc). |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/carla/available_scenarios` | `carla_ros_scenario_runner_types/CarlaScenarioList` | List of available scenarios found in the scenario path. |



## carla_traffic_light_detector

ROS node that detects traffic lights from the Carla simulator and publishes the detection result to a topic in the Autoware-compatible format.


#### Parameters

| Name | Type | Default value | Description |
| ---- | ---- | ------------- | ----------- |
| `/localization/coordinate_transformer` | `string` | `utm` | The coordinate transformer used to load the map. Only "utm" is currently supported. |
| `/localization/utm_origin_lat` | `float` | `0.0` | The latitude of the UTM origin. |
| `/localization/utm_origin_lon` | `float` | `0.0` | The longitude of the UTM origin. |
| `~lanelet2_map_path` | `string` | - | Path of the Lanelet2 map file to load. |
| `/localization/use_custom_origin` | `bool` | `True` | Whether to use a custom origin. |


#### Subscribed Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `/carla/traffic_lights/info` | `carla_msgs/CarlaTrafficLightInfoList` | The Carla traffic light info topic. |


#### Published Topics

| Name | Type | Description |
| ---- | ---- | ----------- |
| `traffic_light_status` | `autoware_mini/TrafficLightResultArray` | The Autoware traffic light status topic. |



## carla_vehicle_interface

This node receives Autoware messages and Carla messages, and publishes AckermannDrive, VehicleStatus, and Float64 messages.


#### Parameters

| Name             | Type   | Default  | Description                                   |
| ---------------- | ------ | -------- | --------------------------------------------- |
| `~max_steer_angle`| `double` | `70.0` | Maximum steering angle in degrees |


#### Subscribed Topics

| Name                     | Type                              | Description                                    |
| ------------------------| ----------------------------------| -----------------------------------------------|
| `/control/vehicle_cmd`   | `autoware_mini/VehicleCmd`        | The control message containing vehicle commands |
| `/carla/ego_vehicle/vehicle_info` | `carla_msgs/CarlaEgoVehicleInfo` | The information message containing vehicle information |
| `/carla/ego_vehicle/vehicle_status` | `carla_msgs/CarlaEgoVehicleStatus` | The status message containing vehicle status |
| `/carla/ego_vehicle/vehicle_control_manual_override` | `std_msgs/Bool` | Message indicating whether the vehicle is under manual control |


#### Published Topics

| Name                             | Type                              | Description                        |
| -------------------------------- | ----------------------------------| -----------------------------------|
| `/carla/ego_vehicle/ackermann_cmd`| `ackermann_msgs/AckermannDrive`   | The ackermann drive command message |
| `/carla/ego_vehicle/target_speed`| `std_msgs/Float64`                | The target speed message (currently used only by scenario runner) |
| `/vehicle/vehicle_status`        | `autoware_mini/VehicleStatus`     | The vehicle status message         |



## carla_waypoints_publisher

Receive a path from carla_ros_waypoint_publisher and convert it to Autoware format.


#### Parameters

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `output_frame` | `string` | - | The output coordinate frame for the converted path. |
| `distance_to_goal_limit` | `float` | - | Distance threshold to determine if the goal has been reached. |
| `speed_limit` | `float` | - | Speed limit for the waypoints in the path. |
| `ego_vehicle_stopped_speed_limit` | `float` | - | Speed threshold to determine if the vehicle is stopped. |
| `/localization/use_custom_origin` | `bool` | `True` | Whether to use a custom origin for UTM coordinates. |
| `/localization/utm_origin_lat` | `float` | `0.0` | Latitude of the custom origin for UTM coordinates. |
| `/localization/utm_origin_lon` | `float` | `0.0` | Longitude of the custom origin for UTM coordinates. |


#### Subscribed Topics

| Name                             | Type              | Description                         |
| -------------------------------- | ----------------- | ----------------------------------- |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current pose of the ego vehicle |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current velocity of the ego vehicle |
| `/carla/ego_vehicle/waypoints`     | `nav_msgs/Path`     | The path from carla_ros_waypoint_publisher |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Goal point set by the user |


#### Published Topics

| Name                  | Type               | Description                    |
| --------------------- | ------------------ | ------------------------------ |
| `lane_change_global_path` | `autoware_mini/Path` | The converted path in Autoware format |
| `/carla/ego_vehicle/goal` | `geometry_msgs/PoseStamped` | The goal point in CARLA coordinates |

#### Services

| Name | Type | Description |
| ---- | ---- | ----------- |
| `cancel_route` | `std_srvs/Empty` | Service to cancel the current route |
