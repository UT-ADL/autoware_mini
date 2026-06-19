# Platform - Monitoring

The monitoring platform in Autoware Mini is responsible for monitoring the health and performance of autonomy-related and sensor ROS topics. It provides real-time visual feedback through dashboard overlays and audio alerts when anomalies or issues are detected, helping safety driver maintain situational awareness and quickly respond to system problems.

## ROS Diagnostic Updater

Many nodes in this folder use the ROS `diagnostic_updater` library to report their status and health. The `diagnostic_updater` automatically periodically publishes diagnostic information to the `/diagnostics` topic using standardized messages (`diagnostic_msgs/DiagnosticStatus`). For more details, see the [official documentation](http://wiki.ros.org/diagnostic_updater).

## cpu_monitor.py
Monitors CPU usage and publishes diagnostics.

#### Parameters
| Name                | Type   | Default Value | Description                                      |
|---------------------|--------|--------------|--------------------------------------------------|
| `~warning_load_average` | `float` (except "auto") | `26`         | Warning threshold for CPU load average. Option "auto" uses the number of logical CPU cores for the threshold. |

#### Published Topics
| Name | Type | Description |
|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Publishes node diagnostics using the ROS [diagnostic_updater](#ros-diagnostic-updater). |

## topic_monitor.py
Monitors the delay and frequency of the autonomy topics.

#### Parameters
##### From YAML or `launch/tools/monitoring.launch` file
| Name                  | Type   | Default Value     | Description                                      |
|-----------------------|--------|------------------|--------------------------------------------------|
| `~monitoring_conf_path` | `string` | `config/monitoring/autonomy.csv` | Path to the monitoring configuration CSV file.    |
| `~ema_gain`             | `float`  | `0.2`            | Exponential moving average gain for topic stats.  |
| `~hardware_id`          | `string` | `topic_monitor`  | Hardware ID for the monitor.                      |
##### Per-topic parameters from CSV file
| Component         | Topic                          | `warning_freq` (Hz) | `error_freq` (Hz) | `warning_delay` (s) | `error_delay` (s) |
|-------------------|-------------------------------|---------------|-------------|----------------|--------------|
| Localization      | `/localization/current_pose`   | `40`          | `30`        | `0.01`         | `0.02`       |
| Object Detection  | `/perception/predicted_objects` | `6`           | `4`         | `0.5`         | `0.75`        |
| Traffic Lights    | `/perception/traffic_light_status` | `5`        | `1`         | `0.15`         | `0.2`        |
| Local Planner     | `/planning/local_path`         | `8`           | `6`         | `0.1`          | `0.15`       |
| Control           | `/control/vehicle_cmd`         | `40`          | `30`        | `0.02`         | `0.03`       |

#### Subscribed Topics
| Name | Type | Description |
|------|------|-------------|
| `/statistics` | `rosgraph_msgs/TopicStatistics` | Receives statistics about message frequency, delay, and delivery for monitored topics. |

#### Published Topics
| Name | Type | Description |
|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Publishes node diagnostics using the ROS [diagnostic_updater](#ros-diagnostic-updater). |

## diagnostics_player.py
Triggers sound notifications.

#### Parameters
| Name           | Type | Default Value | Description                                      |
|----------------|------|--------------|--------------------------------------------------|
| `~sound_loop_rate` | `float` (> 0)  | `20`           | Sets how often (Hz) the node checks for status changes and triggers sound playback. Must be greater than 0. |
| `~components` | `list` of `dict` | see `config/monitoring.yaml` | Each dict contains the substrings of monitored component names as keys. |

#### Subscribed Topics
| Name | Type | Description |
|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Receives diagnostic data. |

## diagnostics_overlay.py
Displays general diagnostic information per component in RViz.

#### Parameters
| Name             | Type  | Default Value | Description |
|------------------|-------|---------------|-------------|
| `~components` | `dict` | see `config/monitoring.yaml` | Keys are substrings of monitored component names and values are display names of these components. |

#### Subscribed Topics
| Name | Type | Description |
|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Receives diagnostic data. |

#### Published Topics
| Name | Type | Description |
|------|------|-------------|
| `/dashboard/diagnostics_overlay` | `jsk_rviz_plugins/OverlayText` | Publishes general diagnostic data to RViz dashboard. |

## detailed_overlay.py
Provides a detailed visual overlay for monitored components in RViz.

#### Parameters
| Name             | Type  | Default Value | Description |
|------------------|-------|---------------|-------------|
| `~components` | `dict` | see `config/monitoring.yaml` | Keys are substrings of monitored component names, values are their dashboard topic names. |

#### Subscribed Topics
| Name | Type | Description |
|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Receives diagnostic data. |

#### Published Topics
| Name | Type | Description |
|------|------|-------------|
| One publisher per component in `component_data`, each on its own dashboard topic (see `config/monitoring.yaml`) | `jsk_rviz_plugins/OverlayText` | Publishes detailed overlay diagnostics for each monitored component. |

## autonomy_overlay.py
Provides a visual diagnostics overlay for autonomy components in RViz.

#### Parameters
| Name             | Type  | Default Value | Description |
|------------------|-------|---------------|-------------|
| `~components` | `dict` | see `config/monitoring.yaml` | Keys are substrings of monitored component names, values are unused (empty strings). |

#### Subscribed Topics
| Name | Type | Description |
|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Receives diagnostic data. |

#### Published Topics
| Name | Type | Description |
|------|------|-------------|
| `/dashboard/autonomy_overlay` | `jsk_rviz_plugins/OverlayText` | Publishes overlay messages containing info about the frequencies and delays of the autonomy components. |
