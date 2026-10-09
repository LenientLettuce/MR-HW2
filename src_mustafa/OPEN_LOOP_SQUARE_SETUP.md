# Open-loop square controller

The implementation is in `scripts/open_loop_square.py`, inside
`OpenLoopSquare`. The ROS package is `a2_description`; its `package.xml`
and `CMakeLists.txt` are directly inside `src_mustafa/`.

## Build inside src_mustafa

In a terminal with ROS 2 Jazzy installed, start at the HW2 / MR-HW2
repository root, then run:

```bash
cd src_mustafa
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select a2_description --cmake-clean-cache
source install/setup.bash
ros2 pkg executables a2_description
```

This creates `build/`, `install/`, and `log/` inside `src_mustafa/`. From
the repository root, source `src_mustafa/install/setup.bash` in every
terminal used for this package. A separate `src/` workspace uses its own
`install/setup.bash`; start a fresh terminal when switching environments.

The previous local artifacts have been restored, but they refer to the old
`src/` location. Run the build above before sourcing them; it clears the
stale CMake cache and regenerates the symlink install for `src_mustafa/`.

`CMakeLists.txt` installs the controller and launch file, and `package.xml`
declares the ROS dependencies. No `setup.py` or separate package is needed.

## Run

Start the simulator in terminal 1, using the desired pair ID:

```bash
ros2 launch a2_description sim.launch.py pair:=P03
```

In terminal 2, source Jazzy and `src_mustafa/install/setup.bash` from the
repository root, then run:

```bash
ros2 run a2_description open_loop_square.py --ros-args \
  -p use_sim_time:=true -p L:=2.0 -p v:=0.2 -p omega:=0.5 \
  -p direction:=ccw
```

| Parameter | Default | Meaning |
|---|---|---|
| `L` | `2.0` | Side length in metres |
| `v` | `0.2` | Forward speed in metres/second |
| `omega` | `0.5` | Turn speed magnitude in radians/second |
| `direction` | `ccw` | `ccw` or `cw` |

Lengths and speeds must be finite and positive. The controller waits for
simulation time and both odometry topics, then drives four straight/turn
pairs. Straight duration is `L / v`; each turn lasts `(pi / 2) / omega`.
Odometry is recorded but does not control motion. After the fourth turn,
the node publishes zero velocity and continues logging until you stop it
with Ctrl+C.

Alternatively, use the defaults with simulation time enabled:

```bash
ros2 launch a2_description open_loop_square.launch.py
```

The launch file supplies only `use_sim_time`; use the direct `ros2 run`
command above to set the controller parameters.

## Recorded data

Each execution creates a unique `square_loop_<uuid>.csv` in the repository's
`Square Loop Runs/` folder. The controller finds the workspace through
`src_mustafa/package.xml` and its own source file. With a symlink install,
this works regardless of the terminal's working directory.

Each incoming `/odom` or `/ground_truth/odom` message gets its own CSV row,
including simulation time, original message stamp, frame IDs, position,
quaternion, and yaw. The two topics are not assumed synchronized. There
is no `csv` parameter; output filenames are generated automatically.

The CSVs are experiment data and remain tracked by Git. The provided
`paths/square.csv` is a reference waypoint file for another exercise,
not a recorded pose log. Plot your runs using
[the graph instructions](../Graphs/README.md).

A full ROS build and simulation run must be checked in your ROS environment.
