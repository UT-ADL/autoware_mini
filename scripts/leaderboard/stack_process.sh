#! /bin/bash -e
map_name=$1

# launch autoware_mini stack for leaderboard evaluation
roslaunch autoware_mini start_carla_leaderboard.launch map_name:=$map_name detector:=lidar_cluster tfl_detector:=camera rviz:=false