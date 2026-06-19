#!/bin/bash

host=$1                    # Carla host
port=$2                    # Carla port
routes_file=$3             # <AUTOWARE_MINI_PKG)/data/routes/tartu_demo.xml
route_id=$4                # 0 for tartu_demo map
agent_file=$5              # <AUTOWARE_MINI_PKG)/nodes/platform/carla/carla_minimal_agent.py
additional_scenarios=$6    # <AUTOWARE_MINI_PKG)/data/routes/<map_name>.py

exec $SCENARIO_RUNNER_ROOT/scenario_runner.py --route $routes_file --route-id $route_id --agent $agent_file --sync --waitForEgo --output --host $host --port $port --timeout 60 --additionalScenario "$additional_scenarios"
