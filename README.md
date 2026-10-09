# MR-HW2

EE/CE 468 Mobile Robotics · Assignment 2

This repository contains independent ROS workspaces. Mustafa's package
`a2_description` and its build environment live inside `src_mustafa/`.
A separate `src/` can have its own build, install, and setup files.

## Contents

| Path | Contents |
|---|---|
| `src_mustafa/` | Mustafa's ROS package and isolated build/install/log folders |
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
