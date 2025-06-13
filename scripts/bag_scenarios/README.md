# Bag scenarios

Bag scenarios allow replaying closed-loop simulation using detections from existing bag recording. They are relatively light-weight and cheap to run, but useful mainly for testing the planner, not so much for testing the perception or control.

The starting point of the simulated vehicle is the starting location in the bag (can be changed with `--start_time` parameter) and the navigation goal is the final location in the bag (can be changed with `--end_time` parameter). The simulated vehicle is visualized as yellow bicycle, the original car location is shown as Lexus model.

The closed-loop simulation can be a bit ahead or behind of the real car location in the bag, but it does not matter for detection as all the detected objects are in `map` frame and therefore independent of the current location of the vehicle. Only if the simulated vehicle gets completely out of detection range of the real vehicle it starts to matter. To avoid this it helps to keep the scenarios short.

## Running bag scenarios

Bag scenarios must be stored in files `data/bag_scenarios/<map_name>/<scenario_name>.bag`. Then they can be launched by including `scenario_name` parameter for `start_sim.launch` (notice that the default for `map_name` parameter is `tartu_demo`).

```
roslaunch autoware_mini start_sim.launch scenario_name:=raekoda
```

Notice that while detected objects come from the bag, the tracking (and potentially prediction) is applied at run time.

## Creating bag scenarios

Bag scenarios can created from any bag file. The only requirement is that the bag files need to contain `/localization/current_pose`, `/localization/current_velocity`, `/detection/detected_objects` and `/detection/traffic_light_status` topics. The latter two can be changed by command line options for special cases. Notice that in current state the detections are taken as they were seen by the car during recording, they are not re-detected from the raw sensor data.

How to create the scenario bag:

```
./create_scenario_bag.py ../data/bags/2023-05-25-14-21-10_sensors_Raekoda.bag ../data/bag_scenarios/tartu_demo/raekoda.bag
```

Optional command line parameters:
* `--start_time <seconds>` - start the scenario after given number of seconds from bag start. Default: None.
* `--end_time <seconds>` - end the scenario after given number of seconds from bag start. Default: None.
* `--goal_delay <seconds>` - delay setting destination goal after setting initial position. Default: 0.1.
* `--detected_objects_topic <topic>` - use this topic for detected objects. Default: /detection/detected_objects.
* `--traffic_light_status_topic <topic>` - use this topic for traffic light status. Default: /detection/traffic_light_status.

For example to force the use of lidar object detections:
```
./create_scenario_bag.py ../../data/bags/2023-05-25-14-21-10_sensors_Raekoda.bag ../../data/bag_scenarios/tartu_demo/raekoda_lidar.bag --detected_objects_topic /detection/lidar/detected_objects
```

Or to force the use of camera traffic light status:
```
./create_scenario_bag.py ../../data/bags/2023-05-25-14-21-10_sensors_Raekoda.bag ../../data/bag_scenarios/tartu_demo/raekoda_camera.bag --traffic_light_status_topic /detection/camera/traffic_light_status
```
## Re-running detection on existing bags

Scenario creation script uses existing detections in the bag file. Sometimes these might be missing or might be inferior to the latest detection stack. To re-record the bag with the latest detection stack you need to run:
```
roslaunch autoware_mini start_bag.launch bag_file:=2023-05-25-14-21-10_sensors_Raekoda.bag record_bag:=raekoda_redetect.bag
```
By default only the topics necessary for creating bag scenarios are recorded. To record all topics include `record_topics:=all` on the command line. You can specify more complex topic inclusion rules as a regular expression, e.g. `record_topics:=(/detection/.*|/localization/.*)`.

You can control also the starting point and duration of the recording with `start:=<seconds>` and `duration:=<seconds>` parameters. These also work when playing bags, not just for recording.

**NB!** You may get different number of detections than originally in the bag. This is just how ROS works. If you have any suggestions, let me know.

## TODO

* Calculate some metrics, e.g. [Carla driving score](https://leaderboard.carla.org/#evaluation-and-metrics) or [RSS](https://carla.readthedocs.io/en/latest/adv_rss/).