# Mission Execution Architecture for Multi-UAV Teams

## Overview

This repository contains an architecture for mission planning and execution in heterogeneous teams of UAVs. The system addresses plan monitoring and execution in the context of Multi-Robot Task Allocation missions and it is implemented in [ROS](https://ros.org/) (Robot Operating System)(MRTA).

This software architecture consists of two layers: a High-Level Planner, centralized on the ground station, and an Agent Behavior Manager, distributed on board each UAV. In this way, the High-Level Planner receives task requests as input, and its work is to solve a resource-constrained problem that allows distributing tasks among the team taking into account vehicles’ capabilities and battery constraints. The Agent Behavior Manager, based on Behavior Trees, is in charge of executing and supervising those plans, calling the appropriate lower-level controllers at any given time. The controllers for each specific action inside each of the tasks are not included in this work, but simple versions of those controllers can be found in this repository in order to be able to test the software layer properly in simulation. Last, replanning operations are triggered in case of unforeseen events, such as vehicle faults or communication drop-outs.

The system is flexible and different modules could be plugged in as High-level Planner as long as they resolve MRTA missions for heterogeneous teams of UAVs. A specific MRTA planner in a separate [repository](https://github.com/multirobot-use/mrta_heuristic_planner), written in Matlab, has been successfully integrated in the architectured contained in the current repository. This Matlab code is connected with ROS 2 through the [matlab_ros2_connector](scripts/matlab_ros2_connector.m) script available in the scripts folder, which implements a ROS 2 Action Server that receives planning and replanning requests from a High-Level Planner.

If you are using this software layer or you found this approach inspiring for your own research, please cite:

```bibtex
@INPROCEEDINGS{CalvoICUAS22,  
  author        = {Calvo, Alvaro and Silano, Giuseppe and Capitan, Jesus},  
  booktitle     = {2022 International Conference on Unmanned Aircraft Systems (ICUAS)},   
  title         = {Mission Planning and Execution in Heterogeneous Teams of Aerial Robots supporting Power Line Inspection Operations},
  year          = {2022},  
  pages         = {1644-1649},
  doi           = {10.1109/ICUAS54217.2022.9836234}
}
```

## Installation

This software has been migrated to Ubuntu with **ROS 2 (Jazzy)** and integrates with
[AEROSTACK2](https://github.com/aerostack2/aerostack2) for UAV platform/estimation/control/
behaviors instead of MAVROS/UAL. To build it:

1. Install ROS 2 Jazzy following the [official instructions](https://docs.ros.org/en/jazzy/Installation.html).

2. Install AEROSTACK2 as a sibling colcon workspace (`~/aerostack2`), following its own
   [installation guide](https://aerostack2.github.io/), including the Gazebo simulation packages
   (`as2_platform_gazebo`, `as2_state_estimator`, `as2_motion_controller`, `as2_behaviors_motion`,
   `as2_gazebo_assets`) and `ros_gz_bridge`.

3. Install the remaining dependencies:

```bash
sudo apt install -y libeigen3-dev libzmq3-dev libboost-dev yaml-cpp
sudo apt install -y ros-jazzy-behaviortree-cpp-v3 ros-jazzy-geographic-msgs
```

4. Create a workspace and clone this repository into `src/`:

```bash
mkdir -p ~/tfg/src
cd ~/tfg/src
git clone https://github.com/lolahzc/tfg_gierm.git mission_planner
git clone https://github.com/multirobot-use/mrta_heuristic_planner.git   # Matlab MRTA solver (optional)
```

5. Build:

```bash
cd ~/tfg
source /opt/ros/jazzy/setup.bash
source ~/aerostack2/install/setup.bash
colcon build --packages-select mission_planner
source install/setup.bash
```

6. Matlab setup (optional, only needed to run the real heuristic planner instead of the
   `heuristic_planner_simulator` faker)

To connect Matlab to ROS 2, install the [ROS Toolbox](https://www.mathworks.com/products/ros.html),
then generate the ROS 2 message classes with the [gen_matlab_msgs_ros2](scripts/gen_matlab_msgs_ros2.m)
script:

```bash
cd ~/tfg/src/mission_planner/scripts
matlab -nodisplay -nosplash -r "gen_matlab_msgs_ros2; exit"
```

You will also need to include the [mrta_heuristic_planner](https://github.com/multirobot-use/mrta_heuristic_planner)
installation folder and subfolders in MATLAB's path, and use the
[matlab_ros2_connector](scripts/matlab_ros2_connector.m) script to run the ROS 2 action server
that handles planning/replanning requests.

## Test

To test that the system works, launch a simulation and inject tasks or unforeseen events via
`ros2` CLI commands, in place of the old Makefile recipes:

```bash
ros2 launch mission_planner mission_simulation.launch.py
...
ros2 run mission_planner gesture_recognition_faker task_1 M human_target_1 1.5 2   # monitor
ros2 run mission_planner gesture_recognition_faker task_2 I 0 7 3 7 7 3            # inspect
ros2 run mission_planner gesture_recognition_faker task_3 D hammer human_target_1  # deliver
...
ros2 topic pub /uav1/battery_fake/control mission_planner/msg/BatteryControl "{mode: 2, value: 0.2, discharge_rate: 0.01, charge_rate: 0.01}"
...
ros2 topic pub /mission_over mission_planner/msg/MissionOver "{value: true}"
...
ros2 node kill /uav1/agent_behaviour_manager   # (or Ctrl-C the process)
ros2 run mission_planner agent_behaviour_manager --ros-args -r __ns:=/uav1
```

For a multi-UAV swarm run, `launch/mission_launch.py` (`n_drones` argument) plus
`scripts/mission_sequencer.py` automate arming/offboard-mode setup and task injection.

## Monitoring the Behavior Tree execution with Groot

Each agent's `agent_behaviour_manager` exposes a BehaviorTree.CPP v3 ZMQ publisher that
[Groot](https://github.com/BehaviorTree/Groot) can connect to, to monitor Behavior Tree execution
in real time or replay a log file.

To monitor a Behavior Tree, specify:

* Server IP: localhost
* Publisher Port: 1666 + (ID - 1) * 2
* Server Port: 1667 + (ID - 1) * 2

E.g., for UAV 1:

* Server IP: localhost
* Publisher Port: 1666
* Server Port: 1667

**Note**: `.fbl` log files are stored in `~/.ros` and are named `bt_trace_uav_` + ID + `.fbl`
