# Localization


## novatel_oem7_localizer

This ROS node transforms GNSS coordinates provided by [NovAtel Oem7 ROS driver](http://wiki.ros.org/novatel_oem7_driver) and converts them to local map coordinates using UTM transformation. The azimuth angle adjustment taking into consideration the meridian convergence is also added. This node publishes the vehicle's current pose, velocity and odometry.


#### Parameters

| Name                    | Type    | Default Value | Description                                                  |
| -----------------------| ------- | ------------- | ------------------------------------------------------------ |
| `parent_frame`          | string  | -             | The name of the parent frame of the published transform. |
| `use_custom_origin`     | bool    | true          | Flag to determine whether to use custom origin or not. If true, origin values in the cartesian coordinate system are subtracted from the coordinates. |
| `utm_origin_lat`        | float   | 0.0           | Latitude of UTM origin point. |
| `utm_origin_lon`        | float   | 0.0           | Longitude of UTM origin point. |
| `use_msl_height`        | bool    | true          | Flag to determine whether to use mean sea level height or ellipsoid height. |
| `offline_height`        | float   | 34.5          | Height value in meters to use when GNSS data is unavailable. |
| `offline_azimuth`       | float   | 185.0         | Azimuth value in degrees to use when GNSS data is unavailable. |
| `offline_lat`           | float   | 58.3854       | Latitude value to use when GNSS data is unavailable. |
| `offline_lon`           | float   | 26.7264       | Longitude value to use when GNSS data is unavailable. |
| `child_frame`           | string  | "base_link"   | The name of the child frame of the published transform. |
| `time_source`           | string  | "auto"        | Time source mode: "ros" (use ROS header time), "gps" (use GPS time), or "auto" (auto-detect based on time sync). |
| `gps_time_leap_seconds` | int     | 18            | GPS leap seconds offset used when converting GPS time to ROS time. |
| `gps_time_sync_threshold`| float  | 1.0           | Max allowed difference (seconds) between GPS and header time for auto-detection. |


#### Subscribed Topics

| Name                 | Type                                         | Description                                                   |
| --------------------| ---------------------------------------------| ------------------------------------------------------------- |
| `/initialpose`       | `geometry_msgs/PoseWithCovarianceStamped`     | Initial pose for relocalization of the vehicle. |
| `/novatel/oem7/bestpos`| `novatel_oem7_msgs/BESTPOS`                 | The best position of the receiver. Undulation (difference between the ellipsoid and mean sea level (geoid) height) is used from this topic. |
| `/novatel/oem7/inspva` | `novatel_oem7_msgs/INSPVA`                  | The INS position, velocity, and attitude. Used fields: header.stamp, latitude, longitude, height, roll, pitch, azimuth, east_velocity, north_velocity |


#### Published Topics

| Name                 | Type                                        | Description                                                   |
| --------------------| --------------------------------------------| ------------------------------------------------------------- |
| `current_pose`       | `geometry_msgs/PoseStamped`                   | The vehicle's current pose in the map frame.                  |
| `current_velocity`   | `geometry_msgs/TwistStamped`                  | The vehicle's current velocity in the map frame.              |
| `odometry`           | `nav_msgs/Odometry`                          | The vehicle's odometry in the map frame.                      |


## novatel_oem7_visualizer

This ROS node visualizes GNSS data provided by the [NovAtel Oem7 ROS driver](http://wiki.ros.org/novatel_oem7_driver) in RViz. It processes INS status, position type, number of satellites, location accuracy, and differential age data to display colored status indicators. The visualizer uses color coding (white for good, yellow for warning, red for bad) to indicate the quality of GNSS data.


#### Parameters

| Name                          | Type    | Default Value | Description                                                   |
| ----------------------------- | ------- | ------------- | ------------------------------------------------------------- |
| `number_of_satellites_good`   | int     | 16            | Threshold for good number of satellites                       |
| `number_of_satellites_bad`    | int     | 8             | Threshold for bad number of satellites                        |
| `location_accuracy_stdev_good`| float   | 0.15          | Threshold for good location accuracy standard deviation (m)   |
| `location_accuracy_stdev_bad` | float   | 0.5           | Threshold for bad location accuracy standard deviation (m)    |
| `differential_age_good`       | float   | 2.0           | Threshold for good differential age (s)                       |
| `differential_age_bad`        | float   | 5.0           | Threshold for bad differential age (s)                        |


#### Subscribed Topics

| Name                  | Type                                         | Description                                                   |
| --------------------- | -------------------------------------------- | ------------------------------------------------------------- |
| `/novatel/oem7/inspva`| `novatel_oem7_msgs/INSPVA`                   | The INS position, velocity, and attitude. Used to determine INS status. |
| `/novatel/oem7/bestpos`| `novatel_oem7_msgs/BESTPOS`                 | The best position of the receiver. Used for satellite info, position type, and accuracy metrics. |


#### Published Topics

| Name            | Type                              | Description                                                   |
| --------------- | --------------------------------- | ------------------------------------------------------------- |
| `gnss_general`  | `jsk_rviz_plugins/OverlayText`    | Summary GNSS status text for RViz display                     |
| `gnss_detailed` | `jsk_rviz_plugins/OverlayText`    | Detailed GNSS status information for RViz display             |
