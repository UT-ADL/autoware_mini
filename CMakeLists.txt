cmake_minimum_required(VERSION 2.8.12...3.25.2)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()
project(autoware_mini)

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   sensor_msgs
   geometry_msgs
   message_generation
   dynamic_reconfigure
   roscpp
   rviz
   pluginlib
)

# Qt5 for RViz plugin
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure/NaiveGroundRemoval.cfg
  config/dynamic_reconfigure/PointsClusterer.cfg
  config/dynamic_reconfigure/EMATracker.cfg
  config/dynamic_reconfigure/CollisionChecker.cfg
)

add_message_files(
  FILES
  DetectedObject.msg
  DetectedObjectArray.msg
  LocalPath.msg
  Log.msg
  Path.msg
  PathArray.msg
  StopLineStatus.msg
  StopLineStatusArray.msg
  VehicleCommand.msg
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
  INCLUDE_DIRS include
  LIBRARIES predicted_trajectory_rviz_plugin detected_objects_rviz_plugin local_path_rviz_plugin heading_aligned_ortho_view_controller
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime dynamic_reconfigure roscpp rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Function to add RViz plugin libraries
# Headers with Q_OBJECT must be listed for AUTOMOC to process them
function(add_rviz_plugin PLUGIN_NAME)
  add_library(${PLUGIN_NAME} ${ARGN})
  target_link_libraries(${PLUGIN_NAME}
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
  )
  add_dependencies(${PLUGIN_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
endfunction()

add_rviz_plugin(predicted_trajectory_rviz_plugin
  include/autoware_mini/rviz/predicted_trajectory_display.h
  src/rviz/predicted_trajectory_display.cpp
  src/rviz/trajectory_visual.cpp
)

add_rviz_plugin(detected_objects_rviz_plugin
  include/autoware_mini/rviz/detected_objects_display.h
  src/rviz/detected_objects_display.cpp
  src/rviz/detected_object_visual.cpp
)

add_rviz_plugin(local_path_rviz_plugin
  include/autoware_mini/rviz/local_path_display.h
  src/rviz/local_path_display.cpp
  src/rviz/local_path_visual.cpp
)

add_rviz_plugin(heading_aligned_ortho_view_controller
  include/autoware_mini/rviz/heading_aligned_ortho_view_controller.h
  src/rviz/heading_aligned_ortho_view_controller.cpp
)

# Install targets
install(TARGETS
  predicted_trajectory_rviz_plugin
  detected_objects_rviz_plugin
  local_path_rviz_plugin
  heading_aligned_ortho_view_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
