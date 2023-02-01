# autoware_mini

Minimalistic python based nodes:
### localizer


### waypoint_saver
* will record waypoint files with columns:
'wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag' 

### waypoint_loader
* imports 2 formats of waypoint files
  * 11 columns: `wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag`
  * 6 columns: `wp_id, x, y, z, yaw, velocity, change_flag`
 

### waypoint_follower