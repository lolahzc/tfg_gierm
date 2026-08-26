# mission_planner_gazebo

AEROSTACK2 configuration and Gazebo bring-up for the simulated swarm. This
package holds no code of its own: it wires up the AEROSTACK2 stack, spawns the
drones and points Gazebo at the world and models that live in
`mission_planner`.

Without it the simulation does not start, so it is versioned alongside the
architecture rather than kept as local setup.

## Launch

```bash
ros2 launch mission_planner_gazebo simulation_launch.py headless:=false
```

Starts Gazebo with the world and drones described in the world file, then
brings up the platform, state estimator, controller and motion behaviours for
every drone listed there. Run `mission_planner`'s own launch afterwards to put
the mission layer on top.

| Argument | Default | Meaning |
|---|---|---|
| `simulation_config_file` | `config/world_swarm.yaml` | World and drones to spawn |
| `headless` | `false` | Run without the Gazebo GUI |
| `use_sim_time` | `true` | Use simulation time |

`platform_launch.py` brings up the stack for a single drone and is included
once per drone by `simulation_launch.py`.

## Configuration

| File | Contents |
|---|---|
| `config/world_swarm.yaml` | Three drones, each spawned on its own charging pad |
| `config/world.yaml` | Single-drone variant |
| `config/config.yaml` | Platform, state estimator, controller and motion behaviour parameters |
| `config/pid_speed_controller.yaml` | PID gains for the speed controller |

### World and models

The world files select `world_name: "evora"`, the Evora solar farm. Neither
that world nor its models ship with AEROSTACK2: they live in
`mission_planner/gazebo_worlds/` and are found because `simulation_launch.py`
adds `share/mission_planner/worlds` and `share/mission_planner/models` to
`GZ_SIM_RESOURCE_PATH`. AEROSTACK2 resolves `world_name` along that same
variable, and Gazebo resolves `model://` URIs the same way.

Drone spawn positions match the `charging_station_droneN` entries in
`mission_planner/config/conf.yaml`, so each drone starts on the pad it will
later return to.

### Battery

`flight_time` is deliberately left unset. Setting it enables Gazebo's
`LinearBatteryPlugin`, which publishes on the same topic as `battery_faker`
but in 0-100 units, while the thresholds in the code are 0-1 fractions.
`battery_faker` owns the battery simulation.

### Middleware

The launch pins `RMW_IMPLEMENTATION` to `rmw_fastrtps_cpp`: CycloneDDS runs
out of participant indices with this many nodes per drone. `mission_planner`'s
launch files pin the same implementation, and both must agree or the two
processes cannot discover each other at all.
