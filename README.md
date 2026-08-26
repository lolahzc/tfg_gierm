# Mission Execution Architecture for Multi-UAV Teams

Mission planning and execution for heterogeneous teams of UAVs, addressing plan
monitoring and execution in the context of Multi-Robot Task Allocation (MRTA)
missions. Built on [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) and
[AEROSTACK2](https://aerostack2.github.io/).

The architecture has two layers:

- **High-Level Planner** — centralised on the ground station. Receives task
  requests, solves a resource-constrained allocation problem that distributes
  tasks across the team according to each vehicle's capabilities and battery,
  and pushes the resulting queues to the agents.
- **Agent Behaviour Manager** — one per UAV. Executes and supervises its
  assigned plan with a Behavior Tree, calling the AEROSTACK2 motion behaviours
  at each step. Replanning is triggered on unforeseen events such as low
  battery or communication drop-outs.

The low-level controllers are not part of this work; simple stand-ins are
included so the software layer can be exercised in simulation.

<video src="./assets/SimulacionCompleta.webm" width="100%" controls></video>

## Repository layout

| Directory | Contents |
|---|---|
| [`mission_planner/`](mission_planner/) | The architecture itself: planner, agents, custom interfaces, simulation stand-ins and Gazebo world assets |
| [`mission_planner_gazebo/`](mission_planner_gazebo/) | AEROSTACK2 configuration and Gazebo bring-up for the simulated swarm |
| [`matlab/`](matlab/) | MATLAB reference material for the heuristic solver. Not part of the build |

Each directory has its own README with the detail.

## Installation

1. Install ROS 2 Jazzy following the
   [official instructions](https://docs.ros.org/en/jazzy/Installation.html).

2. Install AEROSTACK2 as a sibling colcon workspace (`~/aerostack2`) following
   its [installation guide](https://aerostack2.github.io/), including the
   Gazebo packages (`as2_platform_gazebo`, `as2_state_estimator`,
   `as2_motion_controller`, `as2_behaviors_motion`, `as2_gazebo_assets`) and
   `ros_gz_bridge`.

3. Install the remaining dependencies:

   ```bash
   sudo apt install -y libeigen3-dev libzmq3-dev libboost-dev yaml-cpp
   sudo apt install -y ros-jazzy-behaviortree-cpp-v3 ros-jazzy-geographic-msgs
   sudo apt install -y python3-yaml python3-pil
   ```

   `python3-pil` is used to generate the Gazebo world signs at build time.

4. Clone into a workspace and build. The repository holds both packages, so
   cloning it into `src/` is enough:

   ```bash
   mkdir -p ~/tfg/src && cd ~/tfg/src
   git clone https://github.com/lolahzc/tfg_gierm.git

   cd ~/tfg
   source /opt/ros/jazzy/setup.bash
   source ~/aerostack2/install/setup.bash
   colcon build
   source install/setup.bash
   ```

## Running a simulation

Two terminals, each with the workspace sourced:

```bash
# Terminal 1 - Gazebo, the AEROSTACK2 stack and the drones
ros2 launch mission_planner_gazebo simulation_launch.py headless:=false

# Terminal 2 - planner, agents and mission injection
ros2 launch mission_planner mission_launch.py n_drones:=3
```

The mission itself is described in
[`mission_planner/config/mission.yaml`](mission_planner/config/mission.yaml)
and injected wave by wave, so tasks keep arriving while the swarm works.

## Reading the logs

Every agent prints one heartbeat line every 10 s taken straight from
telemetry, which is the quickest way to see what a drone is actually doing:

```
[heartbeat] state=FLYING(4) | z=7.99 m | battery=58% | tasks=1 | telemetry=ok
```

State changes, landings and battery decisions are logged with the values they
were decided on, so a claim in the log can always be checked against the
telemetry behind it:

```
[return-to-base] landing confirmed (z=0.01 m, state=DISARMED)
[battery] 26% below the minimum needed to fly home (26%); aborting task and returning
```

## Reading the simulation

Everything in the world is colour-coded, so the state of the mission can be
read without following the terminal. Three markers are stacked above each
drone, since the models are otherwise identical:

| Marker | Tells you |
|---|---|
| Small sphere on top | Battery: green above 60%, amber down to 30%, red below, at which point it abandons its task and flies home |
| Large sphere | Which drone it is: `drone0` red, `drone1` green, `drone2` blue, matching its charging pad |
| Flat disc underneath | Task type it is running, in the same colour as the waypoint posts it is flying to |

On the ground, each charging pad carries its drone's colour and name, every
task waypoint is a post of its task colour rising to the altitude the drone
will fly to, and the workers and tool station are labelled.

The full legend, including the colour of every task type, is in the
[`mission_planner` README](mission_planner/README.md#reading-the-simulation-in-gazebo).
The markers above the drones need the Gazebo GUI, so launch with
`headless:=false`.

## Citation

If you use this software layer, or found the approach useful for your own
research, please cite:

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
