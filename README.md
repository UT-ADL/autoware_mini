# Autoware Mini

Autoware Mini is minimalistic Python-based autonomy software. It is built on Python and ROS 1 to make it easy to get started and tinkering. It uses Autoware messages to define the interfaces between the modules, aiming to be compatible with Autoware nodes. Autoware Mini currently works on ROS Melodic (Ubuntu 18.04), but we aim to make it compatible also with ROS Noetic (Ubuntu 20.04 and Conda Robostack).

## Goal

We wanted an autonomy stack that
* would be easy to get started with --> minimal amount of dependencies
* would be simple and pedagogical --> simple Python nodes and ROS 1
* would be easy to implement machine learning based approaches --> Python

## Architecture

![Autoware Mini diagram](images/diagram.png)

The key modules of Autoware Mini are:
* **Localization** - determines vehicle position and speed. Can be implemented using GNSS, lidar positioning, visual positioning, etc.
* **Global planning** - given current position and destination determines the global path to the destination. Makes use of HD map.
* **Obstacle detection** - produces detected objects based on lidar, radar or camera information. Includes tracking and prediction.
* **Local planning** - given global path and obstacles plans local path that avoids obstacles and respects traffic lights.
* **Traffic light detection** - produces status for stoplines, if they are green or red. Red stopline is like an obstacle for local planner.
* **Controller** - follows the local path given by the local planner, including target speeds at different points of trajectory.

## Installation

1. Create workspace
   ```
   mkdir -p autoware_mini_ws/src
   cd autoware_mini_ws/src
   ```

2. Clone the repo
   ```
   git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git carla_ros_bridge
   git clone git@gitlab.cs.ut.ee:autonomous-driving-lab/autoware.ai/local/vehicle_platform.git
   git clone git@gitlab.cs.ut.ee:autonomous-driving-lab/autoware_mini.git
   ```

3. Install system dependencies

   ```
   rosdep update
   rosdep install --from-paths . --ignore-src -r -y
   ```

4. Install Python dependencies
   ```
   pip install -r autoware_mini/requirements.txt
   ```

5. Build the workspace
   ```
   catkin build
   ```

## Launching simulation

```
roslaunch autoware_mini start_sim.launch
```

You should see Rviz window with a default map. You need to give the vehicle initial position with 2D Pose Estimate button and goal using 2D Nav Goal button.


## Launching in Lexus

```
roslaunch autoware_mini start_lexus.launch
```

## Launching with Carla

### Download Carla + Tartu map (Skip if already done)

1. Download [Carla 0.9.13](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz) and extract it. We will call this extracted folder `<CARLA ROOT>`.
2. Download [Tartu.tar.gz](https://drive.google.com/file/d/10CHEOjHyiLJgD13g6WwDZ2_AWoLasG2F/view?usp=share_link)
3. Copy Tartu.tar.gz inside the import folder under `<CARLA ROOT>` directory.
4. Run ./ImportAssets.sh from the `<CARLA ROOT>` directory. This will install Tartu map. (You can now delete the Tartu.tar.gz file from import folder.)
5. Since we will be refereing to `<CARLA ROOT>` alot, lets export it as environment variable. Make sure to replace the path where Carla is downloaded.

   ```
   export CARLA_ROOT=/path/to/your/carla/installation
   ```

6. Now, enter the following command. (**NOTE:** Here we assume that `CARLA_ROOT`  was set from the previous command.)
   ```
   export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:${CARLA_ROOT}/PythonAPI/carla/agents:${CARLA_ROOT}/PythonAPI/carla
   ```
   **Note:** It will be convenient if above variables are automatically exported whenever you open a terminal. Putting above exports in `~/.bashrc` will reduce hassle of exporting everytime.

### Launch instructions

1. In a new terminal, (assuming enviornment variables are exported) Run Carla simulator by entering followig command.

   ```
   $CARLA_ROOT/./CarlaUE4.sh -prefernvidia -quality-level=Low
   ```
#### Launch using ground-truth detection:
2. In a new terminal, (assuming enviornment variables are exported) run the following command. This run’s tartu environment of Carla with minimal sensors and our autonomy stack. The detected objects come from Carla directly.

   ```
   roslaunch autoware_mini start_carla.launch
   ```
#### OR
#### Launch using lidar based detector:
2. In a new terminal, (assuming enviornment variables are exported) run the following command. This run’s tartu environment of Carla with lidar sensors and our autonomy stack. The detection is performed using Lidar based euclidean cluster detector.

   ```
   roslaunch autoware_mini start_carla.launch detector:=simple
   ```
#### Generate Traffic
3. In a new terminal, (assuming enviornment variables are exported) generate random traffic by entering following command.

   ```
   python $CARLA_ROOT/PythonAPI/examples/generate_traffic.py --asynch
   ```
