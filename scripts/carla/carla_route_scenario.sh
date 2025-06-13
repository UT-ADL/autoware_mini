#!/bin/bash


routes_file=$1             # <AUTOWARE_MINI_PKG)/data/routes/routes_devtest.xml
route_id=$2                # 0 for tartu_demo map
agent_file=$3              # <AUTOWARE_MINI_PKG)/nodes/platform/carla/carla_minimal_agent.py

$SCENARIO_RUNNER_ROOT/scenario_runner.py --route $routes_file --route-id $route_id --agent $agent_file --sync --waitForEgo --output --timeout 60
