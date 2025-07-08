# #############################
# # Route based scenario Mode
# # Run CARLA leaderboard scenarios and attach an agent to the ego vehicle 
# # 
# #############################

export SCENARIOS=${TEAM_CODE_ROOT}/autoware_mini/data/routes/all_towns_traffic_scenarios_public.json
export ROUTES=${TEAM_CODE_ROOT}/autoware_mini/data/routes/routes_devtest.xml
export REPETITIONS=1
export DEBUG_CHALLENGE=1
export TEAM_AGENT=${TEAM_CODE_ROOT}/autoware_mini/nodes/platform/leaderboard/autoware_mini_agent.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}
export CHECKPOINT_ENDPOINT=${LEADERBOARD_ROOT}/results.json
export CHALLENGE_TRACK_CODENAME=MAP
export AGENT_ROLE_NAME="ego_vehicle"
export RECORD_PATH=${LEADERBOARD_ROOT}/recordings

# Start roscore in background
roscore &

python ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
--scenarios=${SCENARIOS}  \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=${DEBUG_CHALLENGE} \
--record=${RECORD_PATH} \
--resume=${RESUME}
