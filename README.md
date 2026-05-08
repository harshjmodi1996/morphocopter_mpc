# MorphoCopter MPC

This repository contains the implementation of a **Model Predictive Control (MPC)** based planning framework for **MorphoCopter** — a morphing (folding) drone platform. The MPC is implemented using [Acados](https://docs.acados.org/) and integrated with [ROS 2 Humble](https://docs.ros.org/en/humble/) for real-time trajectory optimization, with simulation support via Gazebo Garden and PX4-Autopilot.

---

## Repository Structure

```
morphocopter_mpc/
├── cpp_folding_drone/        # C++ nodes for the folding drone
├── folding_drone/            # Folding drone launch files, configs, and core package
├── folding_drone_msgs/       # Custom ROS 2 message/service definitions
├── gz_models/                # Gazebo Garden model files for the folding drone
├── morphocopter_planning/    # MPC planner, Acados solver, LiDAR viewer, and GCS launch files
├── LICENSE
└── README.md
```

---

## Prerequisites

> **Tested on Ubuntu 22.04**

| Dependency | Version / Notes |
|---|---|
| [Acados](https://docs.acados.org/) (with Python interface) | Latest compatible release |
| [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) | Desktop install recommended |
| [Gazebo Garden](https://gazebosim.org/docs/garden/install) | 7.9 |
| [PX4-Autopilot](https://docs.px4.io/main/en/) | v1.15.2 |
| [pigpio](https://github.com/joan2937/pigpio) | Built from source |
| [px4_msgs](https://github.com/PX4/px4_msgs) | Built via colcon |
| [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) | Built from source |

---

## Installation

### 1. Install Core Dependencies

Make sure **ROS 2 Humble**, **Gazebo Garden 7.9**, **PX4-Autopilot v1.15.2**, and **Acados with its Python interface** are all installed and configured on your system before proceeding.

### 2. Build `px4_msgs`

```bash
cd ~
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
colcon build
```

Then add the workspace to your shell environment:

```bash
echo "source ~/px4_msgs/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install pigpio

```bash
cd ~
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

### 4. Install Micro-XRCE-DDS-Agent

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig
```

### 5. Clone and Build This Repository

Clone `morphocopter_mpc` into your ROS 2 workspace source directory and build with colcon:

```bash
cd ~/<your_ros2_ws>/src
git clone https://github.com/harshjmodi1996/morphocopter_mpc.git
cd ~/<your_ros2_ws>
colcon build
source install/local_setup.bash
```

> **Note:** Replace `<your_ros2_ws>` with the path to your ROS 2 workspace (e.g., `carl_ws`).

---

## Running the MPC Simulation

The simulation requires **multiple terminals**. Open a separate terminal for each step below. Make sure you source your ROS 2 workspace in each terminal.

> **Note:** Replace any instance of `<your_directory>` with the actual path to your workspace source folder.

### Terminal 1 — Start the Micro-XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 — Set Simulation Mode

```bash
ros2 topic pub /folding_drone/in/is_simulation std_msgs/msg/Bool "{data: True}"
```

### Terminal 3 — Bridge Odometry from Gazebo to ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/folding_drone_v2_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

### Terminal 4 — Bridge LiDAR from Gazebo to ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/folding_drone/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

### Terminal 5 — Launch PX4 SITL

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4092 ./build/px4_sitl_default/bin/px4
```

### Terminal 6 — Launch LiDAR Viewer

```bash
ros2 launch morphocopter_planning lidar_viewer.launch.py
```

### Terminal 7 — Launch Ground Control Station with MPC

```bash
ros2 launch morphocopter_planning launch_gcs_mpc.launch.py
```

### Terminal 8 — Run the Acados MPC Node

```bash
cd ~/<your_directory>/morphocopter_planning/acados_mpc
source env/bin/activate
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<your_acados_path>/lib"
export ACADOS_SOURCE_DIR="<your_acados_path>"
ros2 run morphocopter_planning acados_mpc
```

> **Note:** Replace `<your_acados_path>` with the path to your local Acados installation (e.g., `/home/<user>/acados`).

### Terminal 9 — Launch the Folding Drone Simulation

```bash
ros2 launch folding_drone launch.py
```

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## Issues

If you encounter any problems during setup or simulation, please [open an issue](https://github.com/harshjmodi1996/morphocopter_mpc/issues) on this repository.
