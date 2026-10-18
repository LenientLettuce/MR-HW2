# a2_description: the A2 robot

EE/CE 468 Mobile Robotics · Fall 2026 · Assignment 2

This package contains a complete differential-drive robot for Gazebo Sim
(Harmonic) and ROS 2 Jazzy. It includes the robot model, an arena world and
launch files. You use it for every problem in A2. **You do not need to build
the robot yourself.** Problem 1 asks you to understand it.

## 1. What you need installed

This package targets **Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic**. Other
combinations (Humble, Gazebo Classic, Gazebo Fortress) will not work without
changes.

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-tools
```

`ros-jazzy-ros-gz` installs Gazebo Harmonic together with the ROS–Gazebo
bridge.

## 2. Build

```bash
mkdir -p ~/a2_ws/src
cp -r a2_description ~/a2_ws/src/        # or clone the template repo there
cd ~/a2_ws
colcon build --symlink-install
source install/setup.bash                # in EVERY new terminal
```

## 3. Run

Find your pair ID (P01–P12) on Canvas. Every command below uses `pair:=P03`
as the example; **replace it with your own.**

**Inspect the model (no physics):**
```bash
ros2 launch a2_description display.launch.py pair:=P03
```
Drag the wheel sliders and watch the wheel frames turn in RViz.

**Full simulation:**
```bash
ros2 launch a2_description sim.launch.py pair:=P03
```
The terminal prints your robot's physical dimensions and the values the
diff-drive plugin was given.

**Drive it** (second terminal, after sourcing):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

**Launch arguments:**

| Argument | Default | Meaning |
|---|---|---|
| `pair` | `REF` | Your pair ID, `P01`–`P12`, or `REF` (a reference robot with equal wheels) |
| `plugin_separation_scale` | `1.0` | Multiplies **only** the diff-drive plugin's `wheel_separation`. The physical robot is unchanged. Used in *A wrong track width* (extra credit) |
| `lidar_noise` | `0.01` | Standard deviation (m) of the Gaussian noise on every `/scan` range. `0` gives noise-free ranges |
| `plugin_tf` | `true` | Bridge the plugin's `odom → base_link` transform into `/tf`. Set `false` when your own odometry node publishes it (*Your own odometry*) |
| `gt_tf` | `true` | Bridge the ground-truth `world → base_link_gt` transform into `/tf` |
| `gui` | `true` | `false` runs Gazebo without a window (faster; the LiDAR still works) |
| `rviz` | `true` | Open RViz |
| `path` | *(empty)* | `circle` or `square`: draw that reference path in RViz (see §6) |
| `x`, `y`, `yaw` | `0`, `0`, `0` | Spawn pose in the `world` frame (m, m, rad). The wall-following start is `x:=-4.0 y:=-5.2` |
| `cone` | `false` | `true` adds an orange cone 1.5 m ahead of the robot; move it with `cone_driver.py` or teleop (§6) |

## 4. Topics

| ROS 2 topic | Type | Source | Contents |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | you → Gazebo | Commanded `linear.x` (m/s) and `angular.z` (rad/s) |
| `/odom` | `nav_msgs/Odometry` | diff-drive plugin | Odometry computed from the wheel joints **using the plugin's own r and b** |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo | Wheel angles (rad, cumulative) and angular velocities |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR | 360 beams; `ranges[i]` is at angle *i*° counter-clockwise from forward |
| `/ground_truth/odom` | `nav_msgs/Odometry` | physics engine | The **true** pose of `base_link` in the `world` frame |
| `/robot_description` | `std_msgs/String` | robot_state_publisher | The expanded URDF |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | Simulation time |
| `/cone/cmd_vel` | `geometry_msgs/Twist` | you → Gazebo | Cone velocity in its own frame; only with `cone:=true` |
| `/cone/ground_truth/odom` | `nav_msgs/Odometry` | physics engine | True pose of the cone in the `world` frame; only with `cone:=true` |
| `/path` | `nav_msgs/Path` | `path_publisher.py` | Reference path for RViz, in the `odom` frame; only with `path:=circle` or `path:=square`. Read waypoints from `paths/*.csv`, not from this topic |

Run every node you write with `use_sim_time:=true`.

## 5. Frames

```
world ──(spawn pose, static)──► odom ──(/odom, plugin)──► base_link ─┬─► left_wheel_link
  │                                                                ├─► right_wheel_link
  │                                                                ├─► caster_link
  │                                                                └─► lidar_link
  └──(/ground_truth/odom)──► base_link_gt
```

`base_link` is the midpoint of the wheel axle, *r̄* above the ground, with
x forward, y left and z up. `base_link_gt` is where the robot **really** is.
`world → odom` is the spawn pose (the identity with the default spawn), so
`/odom` always starts at zero while `/ground_truth/odom` starts at the spawn
pose. If you spawn away from the origin, express both in the start frame
before comparing them.
In RViz, the gap between `base_link` and `base_link_gt` is your odometry
error.

## 6. Files

| File | Role |
|---|---|
| `urdf/properties.xacro` | Every dimension, defined once. Per-pair values arrive as xacro arguments |
| `urdf/inertia_macros.xacro` | Inertia formulas for a box, a cylinder and a sphere |
| `urdf/a2_bot.urdf.xacro` | Links and joints: chassis, wheels, caster, LiDAR |
| `urdf/a2_bot.gazebo.xacro` | Gazebo-only content: diff-drive, joint states, ground truth, LiDAR, friction |
| `worlds/a2_arena.sdf` | 12 m × 12 m walled arena with a doorway and a buttress |
| `config/pairs.yaml` | Physical and nameplate dimensions for every pair |
| `config/bridge*.yaml` | Which topics cross between Gazebo and ROS 2 |
| `config/*.rviz` | RViz layouts |
| `launch/sim.launch.py` | Starts everything for a pair |
| `launch/display.launch.py` | RViz plus joint sliders, without physics |
| `scripts/umbmark_driver.py` | Drives one UMBmark square, closed loop on your odometry, and logs the return error |
| `scripts/path_publisher.py` | Draws a reference path in RViz by publishing it on `/path` |
| `scripts/cone_driver.py` | Moves the cone: `profile:=line`, `stopgo` or `arc`, at `speed` m/s |
| `models/cone.sdf` | The movable cone; `config/bridge_cone.yaml` bridges its topics |
| `paths/circle.csv`, `paths/square.csv` | Pure-pursuit waypoints, 1 cm apart: columns `x, y, yaw` (m, m, rad) |
| `tools/make_paths.py` | Regenerates `paths/*.csv` (instructor use; not installed) |

**Reference paths for pure pursuit.** Both start at the spawn pose (origin,
facing +x) and run counter-clockwise: `circle.csv` has radius 1.5 m, centred at
(0, 1.5); `square.csv` has side 2 m, corners (0,0), (2,0), (2,2), (0,2). Both
are closed: the last point repeats the first. Load one in your node or your
plotting script with:
```python
import os, numpy as np
from ament_index_python.packages import get_package_share_directory
f = os.path.join(get_package_share_directory('a2_description'), 'paths', 'square.csv')
path = np.loadtxt(f, delimiter=',', skiprows=1)      # N x 3: x, y, yaw
```
To see the path in RViz (yellow), add `path:=circle` or `path:=square` when you launch.

**The moving cone.** Launch with `cone:=true`, then in a second terminal:
```bash
ros2 run a2_description cone_driver.py --ros-args -p use_sim_time:=true \
  -p profile:=line -p speed:=0.2          # or profile:=stopgo, profile:=arc
# or steer it yourself:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cone/cmd_vel
```

To see the URDF that Gazebo actually receives for your pair:
```bash
xacro $(ros2 pkg prefix a2_description)/share/a2_description/urdf/a2_bot.urdf.xacro \
  wheel_radius_left:=<r_L> wheel_radius_right:=<r_R> wheel_separation:=<b> > my_robot.urdf
```

## 7. When something goes wrong

| Symptom | Likely cause and fix |
|---|---|
| `Package 'a2_description' not found` | You did not `source install/setup.bash` in this terminal |
| Gazebo opens but the robot never appears | Check the launch terminal for the spawn error. `ros2 topic echo /robot_description --once` should print XML |
| Robot appears but does not move with teleop | `ros2 topic info /cmd_vel` should show one publisher and one subscriber (the bridge). Check that teleop's terminal has focus |
| RViz: *"Fixed frame [world] does not exist"* | The static `world → odom` publisher has not started yet; wait a few seconds |
| RViz flickers between two robot positions | Two nodes publish `odom → base_link`. Launch with `plugin_tf:=false` when your own odometry node is running |
| `/scan` is empty, or Gazebo crashes on start without a GPU | Try `export LIBGL_ALWAYS_SOFTWARE=1` before launching, or `gui:=false`. On WSL2, make sure WSLg is enabled |
| Simulation runs far slower than real time | Normal without a GPU. Use `gui:=false`. All timing uses simulation time, so results are unaffected |

If Gazebo will not run on your machine after about **two hours** of
trying, stop. Reach out to the TA or instructor.
