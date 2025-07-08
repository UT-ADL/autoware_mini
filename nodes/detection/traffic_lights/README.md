# Detection - Traffic Lights


## mqtt_traffic_light_detector

ROS node that subscribes to MQTT topic, processes received messages and publishes traffic light status. Traffic light status from mqtt service is aggregated into signal groups that directly correspond to stop lines, so no individual traffic light statuses are retrieved and published.


#### Parameters

| Name              | Type    | Default Value | Description                                               |
|-------------------|---------|---------------|-----------------------------------------------------------|
| `~mqtt_host`      | string  |               | Hostname or IP address of the MQTT broker.                |
| `~mqtt_port`      | integer |               | Port number of the MQTT broker.                           |
| `~mqtt_topic`     | string  |               | MQTT topic to subscribe to.                               |
| `~timeout`        | integer |          2.0  | Time (in seconds) before timing out while waiting for a message on the MQTT topic. |
| `~lanelet2_map_name` | string |             | Name of the Lanelet2 map.                                  |
| `/localization/coordinate_transformer` | string | utm | The type of coordinate transformer to use |
| `/localization/use_custom_origin` | bool | false | Whether to use a custom origin or not |
| `/localization/utm_origin_lat` | float | 0.0 | The latitude of the UTM origin |
| `/localization/utm_origin_lon` | float | 0.0 | The longitude of the UTM origin |



#### Subscribed Topics

Subscribes to MQTT Service.


#### Published Topics

| Name                  | Type                                                           | Description                                                            |
|-----------------------|----------------------------------------------------------------|------------------------------------------------------------------------|
| `/traffic_light_status` | `autoware_msgs/TrafficLightResultArray` | Array of traffic light results.
