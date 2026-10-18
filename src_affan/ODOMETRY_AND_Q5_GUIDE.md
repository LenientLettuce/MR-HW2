# Question 5: Odometry Setup and UMBmark Calibration Guide

This guide explains the architecture of the custom odometry pipeline, how the integration schemes work, and the exact step-by-step procedure to execute **Problem 5 (UMBmark Calibration)** in `src_affan`.

---

## 1. System Architecture & Data Flow

```
+-------------------------------------------------------------------------+
|                              GAZEBO SIM                                 |
|  - Simulates robot with P08 ACTUAL physics:                             |
|    r_L = 0.054146 m,  r_R = 0.053854 m,  b = 0.319000 m                 |
|  - Physics engine computes ground truth pose: /ground_truth/odom        |
|  - Simulates wheel encoders and publishes:    /joint_states             |
+-------------------------------------------------------------------------+
                                    |
                             /joint_states
                                    v
+-------------------------------------------------------------------------+
|                      ODOMETRY NODE (odometry_node.py)                   |
|  - Reads wheel angles (phi_L, phi_R) from /joint_states                 |
|  - Computes travel using NAMEPLATE parameters:                          |
|    r_L = r_R = r_n = 0.053 m,  b = b_n = 0.313 m                       |
|    delta_sL = r_n * delta_phiL,   delta_sR = r_n * delta_phiR           |
|    delta_s = (delta_sR + delta_sL) / 2                                  |
|    delta_th = (delta_sR - delta_sL) / b_n                               |
|  - Integrates 2D pose (x, y, theta) using selected scheme (RK2/Euler)   |
|  - Publishes: /odom_own, /odom (alias), and TF (odom -> base_link)      |
+-------------------------------------------------------------------------+
                                    |
                                /odom_own
                                    v
+-------------------------------------------------------------------------+
|                     UMBMARK DRIVER (umbmark_driver.py)                  |
|  - Drives closed-loop based on odometry:                                |
|    * Moves straight until odometry says distance >= L (2.0 m)           |
|    * Rotates in place until odometry says heading turned >= 90 deg      |
|  - Repeats for 4 legs (robot believes it returned to origin (0, 0, 0))  |
|  - Compares ground truth with odometry:                                 |
|    error = (true_pose - odom_pose) = (eps_x, eps_y, eps_theta)          |
|  - Appends result to umbmark.csv                                        |
+-------------------------------------------------------------------------+
```

---

## 2. Odometry Integration Schemes

The assignment allows four methods: **Euler**, **Trapezoidal**, **RK2 (Midpoint)**, and **Exact (Arc)**. 

In `odometry_node.py`, three are implemented:

### A. RK2 / Midpoint (`method:=rk2`) — **Recommended for Question 5**
$$\theta_{mid} = \theta_k + \frac{\Delta \theta}{2}$$
$$x_{k+1} = x_k + \Delta s \cos(\theta_{mid})$$
$$y_{k+1} = y_k + \Delta s \sin(\theta_{mid})$$
$$\theta_{k+1} = \theta_k + \Delta \theta$$
* **Why it is recommended:** It is a 2nd-order Runge-Kutta scheme. Unlike the Exact method, it has **no division by $\Delta \theta$**, meaning it never suffers from division-by-zero or numerical instability when driving straight or standing still.

### B. Euler (`method:=euler`)
$$x_{k+1} = x_k + \Delta s \cos(\theta_k)$$
$$y_{k+1} = y_k + \Delta s \sin(\theta_k)$$
$${\theta}_{k+1} = \theta_k + \Delta \theta$$
* 1st-order method. Simpler, but accumulates slight discretization error during curved motion.

### C. Exact / Arc (`method:=exact`)
$$x_{k+1} = x_k + \frac{\Delta s}{\Delta \theta} \left( \sin(\theta_{k+1}) - \sin(\theta_k) \right)$$
$$y_{k+1} = y_k - \frac{\Delta s}{\Delta \theta} \left( \cos(\theta_{k+1}) - \cos(\theta_k) \right)$$
$${\theta}_{k+1} = \theta_k + \Delta \theta$$
* Models motion as a continuous circular arc. Requires $\Delta \theta \ne 0$.

> **Does Question 5 specify which method to use?**
> **No.** Question 5 simply states: *"Set your odometry node to the nameplate values"*. Since the assignment noted in Question 4(c) that integration methods barely differ on square paths at 50 Hz, **RK2** is the standard, robust choice. You can switch to any method using `method:=exact` or `method:=euler`.

---

## 3. Build & Setup Instructions

Whenever you make changes to `CMakeLists.txt` or start a new shell session:

```bash
cd ~/Documents/Mobile_Robotics/MR-HW2/src_affan
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select a2_description
source install/setup.bash
```

---

## 4. Running the 10 UMBmark Trials

### Terminal 1: Simulation
Start Gazebo with robot P08 and your custom nameplate odometry:
```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/Mobile_Robotics/MR-HW2/src_affan/install/setup.bash
ros2 launch a2_description sim.launch.py pair:=P08 own_odometry:=true
```

### Terminal 2: Square Driver
In a second terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/Mobile_Robotics/MR-HW2/src_affan/install/setup.bash
cd ~/Documents/Mobile_Robotics/MR-HW2
```

> **Crucial Rule:** Before each individual run, press `Ctrl+C` in Terminal 1 and restart `sim.launch.py` so the robot starts cleanly at $(0, 0, 0)$.

1. **Perform 5 Clockwise (`cw`) Runs:**
   ```bash
   ros2 run a2_description umbmark_driver.py --ros-args -p use_sim_time:=true \
     -p odom_topic:=/odom_own -p direction:=cw -p side:=2.0 -p csv:=umbmark.csv
   ```
2. **Perform 5 Counter-Clockwise (`ccw`) Runs:**
   ```bash
   ros2 run a2_description umbmark_driver.py --ros-args -p use_sim_time:=true \
     -p odom_topic:=/odom_own -p direction:=ccw -p side:=2.0 -p csv:=umbmark.csv
   ```

---

## 5. Automated Data Analysis & Plotting

Once all 10 rows are in `umbmark.csv`, run:

```bash
python3 Graphs/plot_umbmark.py
```

This script automatically:
1. Calculates $(x_{c.g., cw}, y_{c.g., cw})$ and $(x_{c.g., ccw}, y_{c.g., ccw})$.
2. Computes the systematic turn error $\alpha$ and straight curvature error $\beta$.
3. Computes the calibrated track width $b'$ and wheel ratio $E_d$.
4. Exports `Graphs/umbmark_plot.png` and `Graphs/umbmark_plot.pdf`.
5. Writes `Graphs/umbmark_report.md` with complete numerical tables for your assignment submission.

