# MR-HW2

EE/CE 468 Mobile Robotics · Assignment 2

This repository contains independent ROS workspaces. Mustafa's package
`a2_description` and its build environment live inside `src_mustafa/`.
Affan's copy of the package (with the Problem 6, 7 and 9 nodes) lives inside
`src_affan/`. Each workspace has its own build, install, and log folders.

## Contents

| Path | Contents |
|---|---|
| `src_mustafa/` | Mustafa's ROS package and isolated build/install/log folders |
| `src_affan/` | Affan's ROS package (`a2_description`) with the extra nodes below |
| `src_affan/scripts/wall_follower.py` | Problem 7 wall-following node (PID on the two-beam wall estimate) |
| `src_affan/scripts/spin_test.py` | Problem 9 spin-in-place test (`/odom` vs ground-truth yaw) |
| `run_q7_experiments.sh` | Runs the Problem 7 experiments (P-only sweep, PD sweep, doorway failure) |
| `analyze_q7.py` | Computes Problem 7 metrics and writes the Problem 7 plots to `Graphs/` |
| `run_q9.sh` | Runs the twelve Problem 9 spin trials (scale 0.9/1.1 x omega 0.5/1.0 x 3 trials) |
| `Q6_b.py` | Problem 6(b) noise propagation calculation |
| `Wall Follower Runs/` | Problem 7 CSV logs (time, D, alpha, e, omega, ranges, ground truth) |
| `Q9 Runs/` | `spin_results.csv`: Problem 9 results (odom yaw, true yaw, ratio) |
| `Pure Pursuit Runs/`, `Arc Runs/` | Problem 8 and arc-run CSV/JSON logs |
| `Square Loop Runs/` | Recorded odometry CSVs; retain these as experiment data |
| `Graphs/` | Plotting script and saved PNG/PDF figures |
| `Graphs/frames/` | Saved TF frame snapshots (`.gv` sources and `.pdf` exports) |
| `Q1c*.jpeg` | Question 1 screenshots |
| `hw2.pdf` | Assignment handout |

## Build and run

In a ROS 2 Jazzy environment, run from this repository root:

```bash
cd src_mustafa
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select a2_description --cmake-clean-cache
source install/setup.bash
ros2 launch a2_description sim.launch.py pair:=P03
```

Replace `P03` with the robot/pair you intend to run. In another terminal,
source Jazzy and `src_mustafa/install/setup.bash` from the repository root,
then follow the
[open-loop square instructions](src_mustafa/OPEN_LOOP_SQUARE_SETUP.md).
See the [package README](src_mustafa/README.md) for simulator options.

## Problem 7: wall following

Build `src_affan` (see below), then from the repository root:

```bash
./run_q7_experiments.sh b      # P-only sweep (Kp = 0.5, 1.0, 1.8)
./run_q7_experiments.sh c      # PD sweep (Kp = 1.0, Kd = 0.5, 1.0, 2.0)
BEST_KP=1.0 BEST_KD=2.0 ./run_q7_experiments.sh e   # doorway failure run
```

Each run launches the simulator at the wall-following pose (`x:=-4.0 y:=-5.2`),
runs `wall_follower.py` for a fixed time and writes a CSV to `Wall Follower Runs/`.
By default the script then runs `analyze_q7.py`, which prints the overshoot,
settling time, steady-state error and turn-rate noise tables and saves the
plots to `Graphs/` (`q7_b_p_only`, `q7_c_pd`, `q7_e_doorway`). Set `GUI=false`
to skip the Gazebo window and `PAIR=...` to change the robot pair (default P08).

## Problem 9: wrong track width

```bash
./run_q9.sh
```

For each plugin separation scale (0.9 and 1.1) and each turn rate (0.5 and
1.0 rad/s), the script relaunches the simulator (`plugin_separation_scale:=...`)
and runs `spin_test.py` three times. The node turns until `/odom` reports one
full revolution, waits 2 s, and records the `/odom` yaw change, the
ground-truth yaw change and their ratio. Results are appended to
`Q9 Runs/spin_results.csv`. Defaults are `PAIR=P08` and `GUI=false`.

Build `src_affan` the same way as `src_mustafa`, from inside its own folder:

```bash
cd src_affan
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Problem 10 (moving target) was not completed.

## Plot saved results

With Matplotlib installed:

```bash
python3 Graphs/plot_square.py
```

See [Graphs/README.md](Graphs/README.md) for other runs and plot options.

## Git layout

The root `.gitignore` covers the entire repository. Build/install/log
directories, Python caches, local environments, editor state, and local
tooling/credentials are ignored at every level. Source CSV waypoints,
recorded CSV runs, screenshots, and curated figures remain versioned.

Build inside `src_mustafa/` so its `build/`, `install/`, and `log/` remain
inside that directory. Build another workspace such as `src/` from inside
its own directory and source its own `install/setup.bash`. Use a fresh
terminal for each environment to avoid mixing packages with the same name.

Rebuild after moving the workspace or package; symlink installs and CMake
caches can contain absolute paths. The build command above clears the old
CMake cache before configuring the current source path.

New `view_frames` exports in the workspace or package root are ignored.
Move snapshots you want to retain into `Graphs/frames/` before adding them.
