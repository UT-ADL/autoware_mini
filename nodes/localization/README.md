# Localization

## novatel_oem7_localizer

* Takes in Novatel ROS topics provided by [NovAtel Oem7 ROS driver](http://wiki.ros.org/novatel_oem7_driver) and converts the coordinates to specified cartesian coordinate frame.
* Adjusts the azimuth angle taking into consideration meridian convergence
* Creates `map` to `base_link` transfom

##### Parameters

* `coordinate_transformer` - into which cartesian coordinate frame he WGS84 latitude and longitude are converted to
  * `utm` - Universal Transvrse Mercator projection. `WGS84ToUTMTransformer.py` file is used to create the coordinate transformer. Internally UtmProjector class from [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_projection) library is used. Origin point is hardcoded in transformer and is used to define the UTM zone.
  * `lest97` - Estonian national coordinae sytem - [read more](https://epsg.io/3301), uses `WGS84ToLest97Transformer.py` file to create the coordinate transformer.
* `use_msl_height` - if true mean sea level height is used, otherwise WGS84 systemuses ellipsoidal height.
* `use_custom_origin` - weather to subtract the origin coordinates from tranformed coordinates or not. If we don't subtract the coordinates the values are too big and cause visualization problems for Rviz.

##### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `/novatel/oem7/inspva` | [novatel_oem7_msgs/INSPVA](https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm) | Used fileds: header.stamp, latitude, longitude, height, roll, pitch, azimuth, east_velocity, north_velocity |
| `/novatel/oem7/bestpos` | [novatel_oem7_msgs/BESTPOS](http://docs.ros.org/en/jade/api/novatel_msgs/html/msg/BESTPOS.html)  | undulation (difference between ellipsoid and mean sea level - geoid - height) |


##### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | pose and orientation of the `base_link` frame |
| `current_velocity` | [geometry_msgs/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) | angular and linear (only linear.x is populatd) velocity |
| `/tf` | tf2_msgs/TFMessage | `map` to `base_link` transform |