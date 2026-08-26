# mission_planner

The mission planning and execution architecture: the centralised planner, the
per-UAV agents, the custom interfaces they speak over, and the stand-ins that
let the whole thing run without real hardware or MATLAB.

## Nodes

| Node | Role |
|---|---|
| `high_level_planner` | Ground station. Tracks connected UAVs through their beacons, keeps the pending task list, and drives allocation through the `HeuristicPlanning` action. Receives task requests on the `NewTask` action and pushes queues to each agent |
| `agent_behaviour_manager` | One per UAV. Runs a BehaviorTree.CPP v3 tree on its own thread, driving the AEROSTACK2 Takeoff/Land/GoToWaypoint behaviours and reporting results back over `TaskResult` |

### Simulation stand-ins

These replace hardware or external software so the architecture can be tested
end to end:

| Node | Replaces |
|---|---|
| `heuristic_planner_simulator` | The MATLAB MRTA solver behind the `HeuristicPlanning` action. Assigns each task to the nearest available agent |
| `battery_faker` | A real battery. Discharges in flight and recharges at a charging pad |
| `ist_ugv_faker` | A ground vehicle, for `MonitorUGV` tasks |
| `gesture_recognition_faker` | The human-gesture interface. A CLI tool that injects a single task request |

## Behaviour tree

`src/behaviour_tree.xml`. `MainTree` decides between returning to base and
running `PerformTaskTree` when the battery allows it and a task is queued,
falling back to `RechargeTree` otherwise. `PerformTaskTree` branches by task
type into the `Monitor`, `MonitorUGV`, `Inspect`, `InspectPVArray`,
`DeliverTool` and `Recharge` subtrees.

Each leaf holds a pointer to its `AgentNode` and reads or mutates its state:
the task queue, position, battery and whether it is carrying a tool.

## Task types

| Type | Task | Parameters |
|---|---|---|
| `M` | Monitor a human target | `human_target_id`, `distance`, `number` |
| `F` | Monitor a UGV | `ugv_id`, `height` |
| `I` | Inspect | any number of waypoints |
| `A` | Inspect a PV array | exactly 2 waypoints |
| `D` | Deliver a tool | `tool_id`, `human_target_id` |
| `R` | Recharge | assigned by the planner, not requested |

## Configuration

| File | Contents |
|---|---|
| `config/conf.yaml` | Named world positions: towers, charging stations, stations, human targets and tools. Read by the planner and by every agent at startup |
| `config/mission.yaml` | The mission plan, grouped into waves with a delay each. See the comments in the file for the fields each task type takes |
| `config/placemarks.yaml` | Additional geographic placemarks |

## Launch

```bash
ros2 launch mission_planner mission_launch.py n_drones:=3
```

Brings up the planner, one `agent_behaviour_manager` and `battery_faker` per
drone, the heuristic stand-in, the mission sequencer and the Gazebo HUD.

| Argument | Default | Meaning |
|---|---|---|
| `n_drones` | `3` | Number of drones |
| `mission_file` | `config/mission.yaml` | Mission plan to inject |
| `config_file` | `config/conf.yaml` | World positions |
| `viz` | `true` | Draw the Gazebo HUD |

`mission_simulation.launch.py` is a single-UAV variant used during
development. It expects the AEROSTACK2 stack to be running already.

## Injecting tasks by hand

```bash
# task id, type, then type-specific parameters
ros2 run mission_planner gesture_recognition_faker task_1 M human_target_1 1.5 2
ros2 run mission_planner gesture_recognition_faker task_2 I 0 7 3 7 7 3
ros2 run mission_planner gesture_recognition_faker task_3 D hammer human_target_1
```

Battery and mission-over events:

```bash
ros2 topic pub /drone0/battery_fake/control mission_planner/msg/BatteryControl \
  "{mode: 2, value: 0.2, discharge_rate: 0.01, charge_rate: 0.01}"
ros2 topic pub /mission_over mission_planner/msg/MissionOver "{value: true}"
```

## Interfaces

The custom messages and actions are the contract between the four moving
pieces. Changing any `.msg` or `.action` needs a full `colcon build` before
either node will compile against it.

| Interface | Direction |
|---|---|
| `AgentBeacon`, `PlannerBeacon` | Liveness, both ways |
| `NewTask` | Task request into the planner |
| `HeuristicPlanning` | Planner to the MRTA solver |
| `NewTaskList` | Planner to agent, task assignment |
| `TaskResult` | Agent to planner, completion or failure |
| `BatteryEnough`, `RequestMobileChargingStation` | Battery bookkeeping |
| `DoCloserInspection` | Follow-up inspection request |

## Visual aids in Gazebo

`scripts/mission_viz.py` draws three markers above each drone, since the
models are otherwise identical in flight:

- a large sphere in the colour of its charging pad,
- a small sphere for the battery: green above 60%, amber down to 30%, red
  below, which is the point at which it abandons its task and heads home,
- a flat disc in the colour of the task type it is running, matching the
  colour of the waypoint posts it is flying to.

This needs the Gazebo GUI: the `/marker` service does not exist in headless
mode. Pass `viz:=false` to disable it.

`scripts/gen_mission_labels.py` generates the ground signs and markers from
`conf.yaml` and `mission.yaml`: labelled charging pads, waypoint posts at the
altitude the drone will fly to, and markers for the workers and tools. It runs
on every build, so editing the mission is enough for the world to follow.
Text is baked into textures because the Ogre2 renderer does not implement
Gazebo's TEXT marker.

## Monitoring with Groot

Each agent exposes a BehaviorTree.CPP v3 ZMQ publisher that
[Groot](https://github.com/BehaviorTree/Groot) can attach to, live or from a
log file:

- Server IP: `localhost`
- Publisher port: `1666 + (ID - 1) * 2`
- Server port: `1667 + (ID - 1) * 2`

Log files are written to `~/.ros` as `bt_trace_uav_<ID>.fbl`.
