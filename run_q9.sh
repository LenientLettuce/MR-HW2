#!/usr/bin/env bash
# run_q9.sh - Problem 9(c): 12 spin tests (scale 0.9/1.1 x omega 0.5/1.0 x 3 trials)
# Usage: ./run_q9.sh      (PAIR=P08 GUI=false by default)
# AI was used to write this script 
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; cd "$REPO_ROOT"
set +u; source /opt/ros/jazzy/setup.bash; source "$REPO_ROOT/src_affan/install/setup.bash"; set -u
PAIR="${PAIR:-P08}"; GUI="${GUI:-false}"
OUT="$REPO_ROOT/Q9 Runs/spin_results.csv"; mkdir -p "$REPO_ROOT/Q9 Runs"; rm -f "$OUT"
clean() { pkill -9 -f rviz2; pkill -9 -f "gz sim"; pkill -9 -f "gz-sim"; pkill -9 -f parameter_bridge
          pkill -9 -f robot_state_publisher; pkill -9 -f static_transform_publisher; sleep 2; } 2>/dev/null
trap clean EXIT INT TERM
for scale in 0.9 1.1; do for w in 0.5 1.0; do for trial in 1 2 3; do
  echo ">>> scale=$scale omega=$w trial=$trial"; clean
  ros2 launch a2_description sim.launch.py pair:="$PAIR" gui:="$GUI" rviz:=false \
      plugin_separation_scale:="$scale" >/tmp/sim_q9.log 2>&1 &
  until timeout 5 ros2 topic echo --once --qos-reliability best_effort /scan >/dev/null 2>&1; do sleep 2; done
  sleep 2
  timeout -s INT 120 ros2 run a2_description spin_test.py --ros-args -p use_sim_time:=true \
      -p omega:="$w" -p csv_path:="$OUT" -p label:="s${scale}_w${w}_t${trial}"
done; done; done
clean; echo; cat "$OUT"
