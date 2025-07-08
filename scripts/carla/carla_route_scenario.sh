#!/bin/bash


routes_file=$1             # <AUTOWARE_MINI_PKG)/data/routes/routes_devtest.xml
scenarios_file=$2          # <AUTOWARE_MINI_PKG)/data/routes/all_towns_traffic_scenarios_public.json
route_id=$3                # 0 for tartu_demo map
agent_file=$4              # <AUTOWARE_MINI_PKG)/nodes/platform/carla/carla_minimal_agent.py

$SCENARIO_RUNNER_ROOT/scenario_runner.py --route $routes_file $scenarios_file $route_id --agent $agent_file --sync --waitForEgo --output
