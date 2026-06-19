
# Platform - Webapp

## webapp_bridge

ROS node bridging Autoware Mini and a web application via MQTT. It synchronizes vehicle state, routes, and goals between vehicle and the webapp, handling session management and secure MQTT communication.

### Parameters (from YAML and launch)

| Name                  | Type    | Default Value                | Description                                      |
|-----------------------|---------|------------------------------|--------------------------------------------------|
| `host`                | string  | `"mqtt.cloud.ut.ee"`         | MQTT broker hostname                              |
| `port`                | int     | `8084`                       | MQTT broker port (websockets)                     |
| `tls_enabled`         | bool    | `True`                       | Enable TLS encryption for MQTT                    |
| `website_public_url`  | string  | `"https://traxi.ut.ee/"`     | Public URL for the webapp                         |
| `session_id`          | string  | `""` (auto-generated)        | Session ID for the webapp bridge                  |

Parameters are loaded from `config/webapp.yaml` and can be overridden via launch file arguments.

### ROS Topics

#### Subscribed

| Name                               | Type                            | Description                                                         |
|------------------------------------|---------------------------------|---------------------------------------------------------------------|
| `/localization/current_pose`       | `geometry_msgs/PoseStamped`     | Current vehicle pose in map coordinates                             |
| `/localization/current_velocity`   | `geometry_msgs/TwistStamped`    | Current vehicle velocity                                            |
| `/planning/global_path`            | `autoware_mini/Path`            | Global path; waypoints forwarded to the webapp as `route`           |
| `/planning/assistance_enabled`     | `std_msgs/Bool`                 | Remote assistance flag; while active the route is frozen for the app|
| `/dashboard/vehicle_drivemode`     | `jsk_rviz_plugins/OverlayText`  | Drive mode; text (HTML stripped) is forwarded as `drivemode_status` |
| `/dashboard/log_message`           | `autoware_mini/Log`             | Planner log; non-instant messages are forwarded as `planner_status` |

#### Published

| Name                            | Type                            | Description                                                               |
|---------------------------------|---------------------------------|---------------------------------------------------------------------------|
| `/move_base_simple/goal`        | `geometry_msgs/PoseStamped`     | Goal point for the vehicle to navigate to                                 |
| `/dashboard/webapp_session_id`  | `jsk_rviz_plugins/OverlayText`  | Session ID overlay for RViz (latched)                                     |
| `/dashboard/webapp_next_stop`   | `jsk_rviz_plugins/OverlayText`  | Name (and district) of the next bus stop on the active goal leg (latched) |

#### Services Used

| Name                   | Type                | Description                                                     |
|------------------------|---------------------|-----------------------------------------------------------------|
| `/planning/get_plan`   | `nav_msgs/GetPlan`  | Called to build preview routes requested over MQTT by the webapp |


### MQTT Communication

#### MQTT Topics

##### Published (bridge → webapp)

| Name                                       | Description                                                                             |
|--------------------------------------------|-----------------------------------------------------------------------------------------|
| `session/<session_id>/status`              | Vehicle status: pose, speed, state, route, chosen spots, route stops, drivemode, planner|
| `session/<session_id>/preview_route`       | Response to a preview route request                                                     |

##### Subscribed (webapp → bridge)

| Name                                         | Description                                                   |
|----------------------------------------------|---------------------------------------------------------------|
| `session/<session_id>/goal`                  | Goal type, target position, chosen spots and route stops      |
| `session/<session_id>/rating`                | User rating and datetime                                      |
| `session/<session_id>/preview_route_request` | Start / end pair for which the webapp wants a planned preview |

#### Vehicle State and Goal Types

##### Vehicle State (`state` field in status topic)

- `waiting`: Idle, waiting for a new goal
- `driving-to-pickup`: Driving to the pickup bus stop
- `arrived-to-pickup`: Arrived at the pickup bus stop
- `driving-to-destination`: Driving from the pickup bus stop to the dropoff bus stop
- `arrived-to-destination`: Arrived at the dropoff bus stop

##### Goal Type (`type` field in goal topic)

- `pickup`: Car should drive to the pickup bus stop
- `dropoff`: Car should drive to the dropoff bus stop
- `done`: End of ride; return to `waiting`

#### MQTT Message Field Details

##### Status Message (`session/<session_id>/status`)

- `pose`:
	- `lat`: Latitude (WGS84)
	- `lng`: Longitude (WGS84)
	- `height`: Altitude in meters
- `speed`: Current vehicle speed in km/h (float, rounded to 2 decimals)
- `state`: Vehicle state (see above)
- `route`: List of 4-tuples `[lat, lng, height, speed_kmh]` — waypoints of the current global path
- `chosen_spots`: Passenger-entered spots, keys `pickup` and `destination`
- `chosen_route_stops`: Selected bus stops, keys `pickup` and `dropoff`
- `drivemode_status`: Text of the drive mode overlay (e.g. `AUTONOMOUS`, `MANUAL`, `ASSIST. DRIVE`)
- `planner_status`: Last non-instant planner log message

##### Goal Message (`session/<session_id>/goal`)

- `type`: Goal type (see above)
- `chosen_spots`: Passenger-entered spots, keys `pickup` and `destination`
- `chosen_route_stops`: Selected bus stops, keys `pickup` and `dropoff`
- `position`:
	- `lat`: Latitude (WGS84)
	- `lng`: Longitude (WGS84)
	- `height`: Altitude in meters

##### Rating Message (`session/<session_id>/rating`)

- `datetime`: ISO 8601 timestamp of rating submission
- `rating`: User rating value (int)

##### Preview Route Request (`session/<session_id>/preview_route_request`)

- `start`, `end`: `{ lat, lng, height }` pairs in WGS84. The bridge calls `/planning/get_plan` and publishes the result to `session/<session_id>/preview_route`.

##### Preview Route Response (`session/<session_id>/preview_route`)

- `route`: List of waypoints `{ lat, lng, height, speed_kmh }` (empty on failure)
- `success`: Boolean — `false` when the planner returned no route

### Session Handling

- Session ID is auto-generated (6 digits) if not provided.
- MQTT topics are namespaced per session.
- Session ID is displayed in Rviz and included in the webapp URL.

### Security

- MQTT credentials are loaded from `.env` in the repo root. See `.env.example` how the `.env` file should look like.
- Node will shut down if credentials are missing.
- TLS is enabled by default for secure MQTT communication.

### Launch & Configuration

- Loads parameters from `config/webapp.yaml`.
- Pass custom session ID via launch argument if needed.

### Ratings

- Ratings received from the webapp are appended to `data/webapp/ratings.csv`.
