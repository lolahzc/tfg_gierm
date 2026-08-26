# MATLAB reference material

Supporting material for the heuristic Multi-Robot Task Allocation solver
studied in this thesis. It is kept here as a record of the work: these files
are **not** part of the ROS 2 build and nothing in `mission_planner` or
`mission_planner_gazebo` depends on them.

The ROS 2 simulation runs against `heuristic_planner_simulator`, which stands
in for this solver behind the `HeuristicPlanning` action, so the system can be
exercised end to end without MATLAB.

## Contents

| Path | What it is |
|---|---|
| `scripts/heuristicTaskAllocator.m` | The heuristic allocator itself |
| `scripts/scenario.m`, `generateScenarios.m`, `generatePlanRepairScenarios.m` | Scenario model and generation |
| `scripts/checkSolution.m`, `computeFval.m`, `printSolution.m`, `previewScenario.m` | Solution checking, cost and inspection |
| `scripts/buildSolutionArray.m`, `extract*.m`, `get*.m`, `saveBestSolutionSoFar.m`, `stopAtFirstValidSolution.m` | Helpers used by the allocator |
| `scripts/matlab_ros2_connector.m` | Bridge that serves `HeuristicPlanning` from MATLAB |
| `scripts/gen_matlab_msgs_ros2.m` | Regenerates the MATLAB bindings for the custom ROS 2 messages |
| `mat/` | Saved agent and task scenarios |

## Running

The scripts use paths relative to `matlab/scripts/`, so run them from that
directory. `gen_matlab_msgs_ros2.m` is the exception: it calls `ros2genmsg`
on the current folder and must be run from the package containing
`package.xml`, that is `mission_planner/`.

The generated bindings land in the separate `matlab_msg_gen` colcon workspace
and must be regenerated whenever a `.msg` or `.action` definition changes.
