cmake_minimum_required(VERSION 2.8.12...3.25.2)
project(autoware_mini)

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   sensor_msgs
   geometry_msgs
   message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ControlCommand.msg
  DetectedObject.msg
  DetectedObjectArray.msg
  Gear.msg
  LampCmd.msg
  Log.msg
  Path.msg
  PathArray.msg
  TrafficLightResult.msg
  TrafficLightResultArray.msg
  VehicleCmd.msg
  VehicleStatus.msg
  Waypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
)
