# Localization


## novatel_oem7_localizer

This ROS node transforms GNSS coordinates provided by [NovAtel Oem7 ROS driver](http://wiki.ros.org/novatel_oem7_driver) and converts them to local map coordinates using either UTM or Lest97 transformations. The azimuth angle adjustment taking into consideration the meridian convergence is also added. Additionally this node publishes vehicle's current pose, velocity and odometry.


##### Parameters

| Name                    | Type    | Default Value | Description                                                  |
| -----------------------| ------- | ------------- | ------------------------------------------------------------ |
| `coordinate_transformer`| string  | "utm"         |  The name of the coordinate transformer to use. Possible values: "utm" and "lest97" |
| `use_custom_origin`     | bool    | true          | Flag to determine whether to use custom origin or not. If true origin values in cartesian coordinate system are subtracted from the coordinates.      |
| `utm_origin_lat`        | float   | -            | Latitude of UTM origin point. Required when `coordinate_transformer` is "utm". |
| `utm_origin_lon`        | float   | -            | Longitude of UTM origin point. Required when `coordinate_transformer` is "utm". |
| `lest97_origin_northing`| float   | -            | Northing of Lest97 origin point. Required when `coordinate_transformer` is "lest97". |
| `lest97_origin_easting` | float   | -            | Easting of Lest97 origin point. Required when `coordinate_transformer` is "lest97". |
| `use_msl_height`        | bool    | true         | Flag to determine whether to use mean sea level height or ellipsoid height. |
| `child_frame`           | string  | "base_link"  | The name of the child frame of the published transform. |


`coordinate_transformer`:
  * `utm` - Universal Transvrse Mercator projection. `WGS84ToUTMTransformer.py` file from helpers/localization is used to create the coordinate transformer. Internally UtmProjector class from [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_projection) library is used. Origin point is defined in the `localization.yaml` file
  * `lest97` - Estonian national coordinate system - [read more](https://epsg.io/3301), uses `WGS84ToLest97Transformer.py` from helpers/localization. Origin northing and easting are defined in the `localization.yaml` file


##### Subscribed Topics

| Name                 | Type                                         | Description                                                   |
| --------------------| ---------------------------------------------| ------------------------------------------------------------- |
| `/novatel/oem7/bestpos`| [`novatel_oem7_msgs/BESTPOS`](http://docs.ros.org/en/jade/api/novatel_msgs/html/msg/BESTPOS.html)                 | The best position of the receiver. Undulation (difference between ellipsoid and mean sea level - geoid - height) is used from this message. |
| `/novatel/oem7/inspva` | [`novatel_oem7_msgs/INSPVA`](https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm)                  | The INS position, velocity, and attitude. Used fileds: header.stamp, latitude, longitude, height, roll, pitch, azimuth, east_velocity, north_velocity |


##### Published Topics

| Name                 | Type                                        | Description                                                   |
| --------------------| --------------------------------------------| ------------------------------------------------------------- |
| `current_pose`       | [`geometry_msgs/PoseStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)                   | The vehicle's current pose in the map frame.                  |
| `current_velocity`   | [`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)                  | The vehicle's current velocity in the map frame.              |
| `odometry`           | `nav_msgs/Odometry`                          | The vehicle's odometry in the map frame.                      |
