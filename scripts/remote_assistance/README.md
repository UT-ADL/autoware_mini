# Remote assistance Scripts

## Overview
This directory contains scripts related to the remote assistance of the self-driving car. These scripts are intended to be run on the remote operator's computer, not on the vehicle itself. The operator's computer has a VPN connection to the car. The scripts in this folder provide tools for remotely assisting the car when manual intervention is required.

## Prerequisites

NVIDIA Drive must be running and the camera application must be configured to auto-start on the NVIDIA Drive (see wiki: [Configure auto-start of camera application on NVIDIA Drive](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware_mini/-/wikis/Configure-auto-start-of-camera-application-on-NVIDIA-Drive)).

## Setup

Each component has its own `setup.sh` (which installs that folder's
`requirements.txt`), so you can set up only the ones you need. Before first use,
run the `setup.sh` for every component you intend to launch:

``` bash
./button_controller/setup.sh   # StreamDeck button controller
./wheel_ffb/setup.sh           # wheel force-feedback daemon
```

`button_controller/setup.sh` installs the StreamDeck Python deps and adds the
StreamDeck udev rule. `wheel_ffb/setup.sh` installs the FFB deps and adds your
user to the `input` group (so the daemon can read the wheel) — log out and back
in once afterwards for that group change to take effect.

## Running

Launch remote RViz, the NVIDIA camera web view, the StreamDeck button controller, and the wheel force-feedback daemon:

``` bash
./remote_assistance.sh
```

The script opens a browser to view the cameras (the camera application runs on NVIDIA Drive at startup, see prerequisites), launches the StreamDeck button controller (`button_controller/button_controller.py`) and the wheel force-feedback daemon (`wheel_ffb/wheel_ffb.py`), and starts RViz.

Optional arguments: `./remote_assistance.sh [ROS_MASTER_IP] [NVIDIA_IP] [CAMERA_PORT]` (defaults: `192.168.100.100`, `192.168.100.200`, `8080`).

## Button Controller

### Setup instructions

#### First time setup on the Clevon's PC

Install the button controller's dependencies once with `./button_controller/setup.sh`
(see the **Setup** section above).

#### For testing in simulation

1. In the host machine:

    a. Navigate to this folder

    b. Set the `ROS_IP` environment variable with the host visible IP value:
    
    ``` bash
    export ROS_IP=<host_visible_IP>
    ```

    c. Start simulation with manual yield line checking (on the same terminal!):
    ``` bash
    roslaunch autoware_mini start_sim.launch map_name:=tartu_large enable_manual_yield_checker:=true
    ```

2. In Clevon PC:

    Set `ROS_MASTER_URI` environment variable with the host's visible IP address:

    ``` bash
    export ROS_MASTER_URI="http://<host_visible_IP>:11311"
    ```

#### For testing with the real car

1. In Clevon PC, set the `ROS_MASTER URI` environment variable to `http://192.168.100.100:11311`:

    ``` bash
    export ROS_MASTER_URI="http://192.168.100.100:11311"
    ```

2. In Clevon PC, set the `ROS_IP` environment variable to `10.0.1.2`

### Running instructions

1. Navigate to `button_controller` on the terminal.

2. Run the script:

    a. In normal mode:

    ``` bash
    ./button_controller.py
    ```

    b. In verbose mode:

    ``` bash
    ./button_controller.py --verbose
    ```

## Wheel force feedback

The operator's steering wheel is driven by the force-feedback daemon in
`wheel_ffb/` (`wheel_ffb.py`), which `remote_assistance.sh` starts
automatically — it connects to the browser teleop UI over a localhost WebSocket
and drives the wheel to the car's reported steering angle. Per-vendor PID /
friction presets live in `wheel_ffb/wheel_configs.json`.

Its dependencies (`evdev`, `aiohttp`, `docopt`) and the `input`-group membership
it needs to read the wheel are installed by `./wheel_ffb/setup.sh` (see the
**Setup** section).
