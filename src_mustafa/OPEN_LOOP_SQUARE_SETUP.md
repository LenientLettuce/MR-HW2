# Where to write your node

Write your solution in `scripts/open_loop_square.py`, inside `OpenLoopSquare`.
This file was newly added: there was no open-loop square implementation or
starter file in the supplied package. The scaffold provides imports, the node
name, and initialization/spin/shutdown only. Motion, parameters, subscriptions,
publishing, and CSV logging are yours to implement.

`src/` in this repository is itself the ROS package `a2_description`:
its `package.xml` and `CMakeLists.txt` are directly inside it. It uses
`ament_cmake`, even though the helper nodes are Python scripts. You do not
need a new package, `setup.py`, `setup.cfg`, or custom message definitions.

## Wiring already done

- `CMakeLists.txt` installs your script beside the existing helper nodes, so
  `ros2 run a2_description open_loop_square.py` can find it after a build.
- The script has a Python shebang and executable permission.
- `package.xml` already declares `rclpy`, `geometry_msgs`, and `nav_msgs`.
  Explicit `launch` and `launch_ros` runtime dependencies were added.
- `launch/open_loop_square.launch.py` starts only your node with
  `use_sim_time=True`. The existing CMake directory-install rule installs it.
- `config/bridge.yaml`, loaded by `sim.launch.py`, already bridges `/cmd_vel`,
  `/odom`, `/ground_truth/odom`, and `/clock`. No bridge edits are needed.

## Imports and existing code

| Import | Use |
|---|---|
| `import rclpy` | Initialize, spin, and shut down ROS |
| `from rclpy.node import Node` | Base class; parameters, publishers, subscribers, timers, clock |
| `from geometry_msgs.msg import Twist` | `/cmd_vel`: `linear.x` and `angular.z` |
| `from nav_msgs.msg import Odometry` | Both odometry topics: `msg.pose.pose.position` and `.orientation` |
| `import csv` | Standard-library CSV writer; no ROS dependency needed |
| `import math` | Angles and quaternion/yaw calculations; standard library |

You do not need to import any local script. `umbmark_driver.py` contains
`yaw_of`, `pose_of`, `wrap`, and `relative`, which you may inspect as references.
Its `yaw_of` assumes planar yaw-only orientation. Its `UMBmarkDriver` uses
odometry feedback and writes a final error row, so it is not the requested
open-loop controller or a continuous pose logger. `cone_driver.py` shows the
existing node-clock/timer conventions. Neither is a shared utility package;
there is no need to wire your node to them.

Use `self.get_clock()` for your timing. ROS supplies `use_sim_time`; you do
not need to declare it yourself. The node needs the running simulator's
`/clock` to advance when simulation time is enabled. The CSV output location
is your choice; a relative filename is relative to the terminal's working
directory. The provided `paths/square.csv` is a reference waypoint file for
another exercise, not your pose log.

## Build and run in a ROS 2 environment

The supplied package targets ROS 2 Jazzy with Gazebo Harmonic on Ubuntu 24.04
(see `README.md`). This Codex session has no `/opt/ros/jazzy`, `ros2`, or
`colcon`, so the ROS build and runtime have not been verified here. Existing
`src/build` and `src/install` directories are old build artifacts; their
presence does not establish that ROS is installed in the current environment.

In a terminal where Jazzy is installed, build from the repository root:

```bash
cd '/home/mustafa/Mustafa/College Uni/Habib/Sem7/MR/HW/HW2'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select a2_description
source install/setup.bash
ros2 pkg executables a2_description
```

Building here creates `build/`, `install/`, and `log/` at the repository root.
Use that root `install/setup.bash` for these commands, rather than the old
`src/install/setup.bash`. No files need to be moved.

Start the simulation in terminal 1, substituting your assigned pair ID:

```bash
ros2 launch a2_description sim.launch.py pair:=P03
```

In terminal 2, source Jazzy and this workspace's root `install/setup.bash`, then:

```bash
ros2 run a2_description open_loop_square.py --ros-args -p use_sim_time:=true
```

Alternatively, the new launch file enables simulation time automatically:

```bash
ros2 launch a2_description open_loop_square.launch.py
```

The scaffold will spin and print its TODO message; it does not move the robot
or create a CSV. Once you declare parameters with the names below, you can run:

```bash
ros2 run a2_description open_loop_square.py --ros-args \
  -p use_sim_time:=true -p L:=2.0 -p v:=0.15 -p omega:=0.4 \
  -p direction:=ccw -p csv:=square_run.csv
```

`L`, `v`, `omega`, `direction`, and `csv` are suggested names for your
implementation, not parameters implemented by this scaffold. Define your
accepted direction strings consistently, for example `ccw` and `cw`.
Use the direct `ros2 run` command to supply your parameters; the new launch
file currently supplies only `use_sim_time`.

Rebuild after adding scripts or changing package/launch installation rules.
With `--symlink-install`, edits to this installed Python script are normally
picked up on its next run.
