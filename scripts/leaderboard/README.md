
## Launching Leaderboard

With this stack you can run Carla's leaderboard evaluations on your local machine. Follow the steps below: 

(<b>NOTE: Currently only leaderboard-1.0 is supported </b>)

1. Download the Leaderboard repository and checkout to specified commit.
   ```
   git clone https://github.com/carla-simulator/leaderboard.git && cd leaderboard &&
   git checkout f731410c23efc6ef329a7810b44a71961cee3174
   ```
2. Point environment variable `LEADERBOARD_ROOT` to the downloaded location of leaderboard.
   ```
   export LEADERBOARD_ROOT=<path_to>/leaderboard
   ```
3. Install the required Python dependencies.
   ```
   cd $LEADERBOARD_ROOT
   pip3 install -r requirements.txt
   ```
4. Make sure you have downloaded scenario runner and exported its root path to `SCENARIO_RUNNER_ROOT`. Kindly see Launching with [Scenario Runner section](../../README.md#launching-with-scenario-runner) for details.

5. We need to make sure that different modules find each other. Following environment variables should be set in `.bashrc`.

   ```
   export CARLA_ROOT=PATH_TO_CARLA_ROOT
   export SCENARIO_RUNNER_ROOT=PATH_TO_SCENARIO_RUNNER
   export LEADERBOARD_ROOT=PATH_TO_LEADERBOARD
   export TEAM_CODE_ROOT=PATH_TO_SRC_FOLDER_OF_OUR_ROS_WORKSPACE
   ```
   once you fill in paths to above variables, paste following line too in `.bashrc`

   ```
   export PYTHONPATH="${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:${CARLA_ROOT}/PythonAPI:${CARLA_ROOT}/PythonAPI/carla/agents:${CARLA_ROOT}/PythonAPI/carla${PYTHONPATH}
   ```

6. Finally, run leaderboard evaluations as follows:

   In a separate terminal
   ```
   $CARLA_ROOT/CarlaUE4.sh
   ```

   In a separate terminal
   ```
   roscd autoware_mini/scripts/leaderboard/

   ./run_evaluations.sh
   ```

## Analyzing Replay

You can analyze the replay of the leaderboard evalautions. For this you need replay log file(s) which are currently saved under `$LEADERBOARD_ROOT/recordings`.

1. Copy replay file from neuron server

   ```
   scp leaderboard@neuron.hpc.ut.ee:/home/leaderboard/github/leaderboard/recordings/<NAME_OF_LOG_FILE> .
   ```

   OR use the ones saved on your machine.

2. In a separate terminal
   ```
   $CARLA_ROOT/CarlaUE4.sh
   ```

3. `(Optional)` Get ID of the actor to attach spectator camera. In Carla simulator ego_vehicle is usually addressed by the actor name `hero`.

   ```
   python $CARLA_ROOT/PythonAPI/examples/show_recorder_file_info.py -f <PATH_TO_LOG_FILE>
   ```

4. In a separate terminal.

   ```
   python $CARLA_ROOT/PythonAPI/examples/start_replaying.py -f <PATH_TO_LOG_FILE> -c <ID_OF_ACTOR_TO_FOCUS>
   ```