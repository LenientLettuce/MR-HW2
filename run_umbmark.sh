#!/usr/bin/env bash
# run_umbmark.sh - Automated runner for Question 5 UMBmark (5 CW + 5 CCW)
# Gemini was used to help create this file 
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/src_affan/install/setup.bash"

CSV_FILE="$REPO_ROOT/umbmark.csv"
rm -f "$CSV_FILE"

clean_sim() {
    pkill -9 -f rviz2 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gz-sim" 2>/dev/null || true
    pkill -9 -f parameter_bridge 2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f static_transform_publisher 2>/dev/null || true
    pkill -9 -f odometry_node 2>/dev/null || true
    sleep 2
}

echo "=========================================================="
echo "          STARTING AUTOMATED UMBMARK EXPERIMENTS"
echo "=========================================================="
clean_sim

# 5 Clockwise Runs
for i in {1..5}; do
    echo ""
    echo ">>> Running CW Trial $i of 5 (with Gazebo & RViz GUI)..."
    ros2 launch a2_description sim.launch.py pair:=P08 own_odometry:=true &
    SIM_PID=$!
    
    # Wait for Gazebo GUI and RViz to initialize
    sleep 6
    
    ros2 run a2_description umbmark_driver.py --ros-args -p use_sim_time:=true \
      -p odom_topic:=/odom_own -p direction:=cw -p side:=2.0 -p csv:="$CSV_FILE" || true
    
    kill $SIM_PID 2>/dev/null || true
    clean_sim
done

# 5 Counter-Clockwise Runs
for i in {1..5}; do
    echo ""
    echo ">>> Running CCW Trial $i of 5 (with Gazebo & RViz GUI)..."
    ros2 launch a2_description sim.launch.py pair:=P08 own_odometry:=true &
    SIM_PID=$!
    
    # Wait for Gazebo GUI and RViz to initialize
    sleep 6
    
    ros2 run a2_description umbmark_driver.py --ros-args -p use_sim_time:=true \
      -p odom_topic:=/odom_own -p direction:=ccw -p side:=2.0 -p csv:="$CSV_FILE" || true
    
    kill $SIM_PID 2>/dev/null || true
    clean_sim
done

echo ""
echo "=========================================================="
echo "      ALL 10 TRIALS COMPLETE! RUNNING ANALYSIS..."
echo "=========================================================="
python3 Graphs/plot_umbmark.py --csv "$CSV_FILE"

