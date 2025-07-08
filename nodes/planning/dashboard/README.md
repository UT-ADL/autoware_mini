# Planning - Logger

## Logger

The `Logger` node is responsible for throttling and formatting log messages for visualization in RViz.

#### Parameters

| Name                | Type   | Default Value | Description                                                                 |
|---------------------|--------|---------------|-----------------------------------------------------------------------------|
| `~throttle`         | float  | `2.0`         | Seconds to wait before publishing the same message again.                  |
| `~history_length`   | int    | `5`           | Number of log messages to keep in the history for display.                 |

#### Subscribed Topics

| Name           | Type               | Description                              |
|----------------|--------------------|------------------------------------------|
| `log_message`  | `autoware_mini/Log` | Receives log messages to be processed.  |

#### Published Topics

| Name         | Type                   | Description                              |
|--------------|------------------------|------------------------------------------|
| `log_text`   | `jsk_rviz_plugins/OverlayText` | Publishes formatted log messages for RViz. |
