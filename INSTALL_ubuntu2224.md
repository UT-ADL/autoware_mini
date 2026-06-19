# Installing Autoware Mini on Ubuntu 22.04 / 24.04

The main [README](README.md) covers the default setup on Ubuntu 20.04 (ROS Noetic, CUDA 11). This guide covers **Ubuntu 22.04 and 24.04**, which use [ROS One](https://ros.packages.techfak.net/) and CUDA 12. The two are nearly identical; where a command differs, separate copy-paste blocks are given for each.

## Prerequisites

1. Install ROS One by following the instructions at https://ros.packages.techfak.net/. After installing, source the setup file:
   ```
   source /opt/ros/one/setup.bash
   ```

2. Some of the nodes need NVIDIA GPU, CUDA, cuDNN and TensorRT. **Notice that the default setup also runs without GPU.**

   **Ubuntu 22.04:**
   ```
   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
   sudo dpkg -i cuda-keyring_1.1-1_all.deb
   sudo apt-get update
   sudo apt-get install cuda-toolkit-12-9 libcudnn9-cuda-12 libnvinfer10=10.16.1.11-1+cuda12.9 libnvonnxparsers10=10.16.1.11-1+cuda12.9
   ```
   **Ubuntu 24.04:**
   ```
   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
   sudo dpkg -i cuda-keyring_1.1-1_all.deb
   sudo apt-get update
   sudo apt-get install cuda-toolkit-12-9 libcudnn9-cuda-12 libnvinfer10=10.16.1.11-1+cuda12.9 libnvonnxparsers10=10.16.1.11-1+cuda12.9
   ```

## Installation

1. Create the workspace
   ```
   mkdir -p ~/autoware_mini_ws/src
   cd ~/autoware_mini_ws/src
   ```

2. Clone the `autoware_mini` repo and the ROS dependencies missing from the ROS One repositories
   ```
   git clone https://github.com/UT-ADL/autoware_mini.git
   git clone https://github.com/novatel/novatel_oem7_driver.git -b 4.3.0
   git clone https://github.com/swri-robotics/gps_umd.git -b 0.3.4
   git clone https://github.com/ros-drivers/nmea_msgs.git -b master
   git clone https://github.com/ros-drivers/ackermann_msgs.git -b ros1
   git clone https://github.com/astuff/astuff_sensor_msgs.git -b master
   git clone https://github.com/ros-perception/radar_msgs.git -b noetic
   git clone https://github.com/ros-geographic-info/unique_identifier.git -b master
   git clone https://github.com/astuff/automotive_autonomy_msgs.git -b master
   git clone https://github.com/KIT-MRT/mrt_cmake_modules.git -b master
   git clone https://github.com/fzi-forschungszentrum-informatik/Lanelet2.git -b master
   git clone https://github.com/carla-simulator/ros-carla-msgs.git carla_msgs
   ```

3. Update the `CMakeLists.txt` in the `novatel_oem7_driver` package. Open `~/autoware_mini_ws/src/novatel_oem7_driver/src/novatel_oem7_driver/CMakeLists.txt` and replace
   ```
   add_compile_options(-std=c++11)
   ```
   with
   ```
   add_compile_options(-std=c++17)
   ```

4. Install system dependencies (ignore the errors for missing Carla packages if not using Carla). Running it twice is intended.
   ```
   rosdep update
   rosdep install --include-eol-distros --from-paths . --ignore-src -r -y
   rosdep update
   rosdep install --include-eol-distros --from-paths . --ignore-src -r -y
   ```

5. Build the workspace. This must be done in `Release` mode for Lanelet2 to be fast.
   ```
   cd ..
   catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

6. Install the prerequisite system package for Python audio and upgrade pip
   ```
   sudo apt install libasound2-dev
   pip install --upgrade pip
   ```

7. Install Python dependencies

   **Ubuntu 22.04:**
   ```
   pip install --user -r autoware_mini/requirements_ubuntu2224.txt
   # only when planning to use GPU-based clustering, long download
   pip install --user -r autoware_mini/requirements_cuda12.txt
   ```
   **Ubuntu 24.04** (`--break-system-packages` required by [PEP 668](https://peps.python.org/pep-0668/)):
   ```
   pip install --user --break-system-packages -r autoware_mini/requirements_ubuntu2224.txt
   # only when planning to use GPU-based clustering, long download
   pip install --user --break-system-packages -r autoware_mini/requirements_cuda12.txt
   ```

8. Source the workspace environment
   ```
   source devel/setup.bash
   ```
   As this needs to be run every time before launching the software, you might want to add it to your `~/.bashrc`.
   ```
   echo "source ~/autoware_mini_ws/devel/setup.bash" >> ~/.bashrc
   ```
