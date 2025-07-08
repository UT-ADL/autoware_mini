# Platform - Webapp

## webapp_bridge

ROS node that bridges communication between Autoware Mini and a web application using MQTT protocol. It transforms and exchanges data between ROS topics and MQTT topics.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `host` | string | `"mqtt.cloud.ut.ee"` | MQTT broker hostname |
| `port` | int | `8883` | MQTT broker port |
| `tls_enabled` | bool | `True` | Whether TLS encryption is enabled for MQTT connection |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current vehicle pose in map coordinates |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current vehicle velocity |
| `/planning/global_path` | `autoware_mini/Path` | Global path for the vehicle to follow |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Goal point for the vehicle to navigate to |

#### MQTT Communication

##### Published MQTT Topics
| Name | Description |
| ----- | ------------ |
| `lexus/status` | Vehicle status information including position and speed |
| `lexus/route` | Current route as a list of waypoints |

##### Subscribed MQTT Topics
| Name | Description |
| ----- | ------------ |
| `lexus/goal` | Goal point for navigation in WGS84 coordinates |
