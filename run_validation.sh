#!/usr/bin/env bash
# run_validation.sh - Task 5(f): validate the calibrated odometry with UMBmark squares.
# Usage: ./run_validation.sh [RUNS_PER_DIRECTION]     (default 1, as the handout asks)
# Each run first VERIFIES that the odometry node really loaded the calibrated parameters.
set -u

N_RUNS="${1:-1}"
PAIR="P08"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

set +u                               # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/src_affan/install/setup.bash"
set -u

CSV_VAL="$REPO_ROOT/umbmark_calibrated.csv"
UMB_CSV="$REPO_ROOT/umbmark.csv"
CALIB_YAML="$REPO_ROOT/src_affan/config/odometry_calibrated.yaml"

if [ ! -f "$CALIB_YAML" ]; then
    echo "ERROR: $CALIB_YAML not found. Run scale_calibrate.py first."
    exit 1
fi
rm -f "$CSV_VAL"

clean_sim() {
    for p in rviz2 "gz sim" gz-sim parameter_bridge robot_state_publisher \
             static_transform_publisher odometry_node; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
    sleep 2
}

wait_for_topic() {
    for _ in $(seq 1 60); do
        if timeout 5 ros2 topic echo "$1" --once >/dev/null 2>&1; then return 0; fi
        sleep 1
    done
    return 1
}

# Compare the running node's r_left, r_right, b with the values in the calibrated YAML.
verify_params() {
    local node ok=0 p got want
    node=$(timeout 10 ros2 node list 2>/dev/null | grep -m1 odometry)
    if [ -z "$node" ]; then
        echo "    ERROR: no odometry node found, cannot verify parameters"
        return 1
    fi
    for p in r_left r_right b; do
        got=$(timeout 10 ros2 param get "$node" "$p" 2>/dev/null | awk '{print $NF}')
        want=$(awk -v k="$p:" '$1==k {print $2}' "$CALIB_YAML")
        if [ -z "$got" ] || [ -z "$want" ]; then
            echo "    ERROR: could not read parameter $p (node $node)"; ok=1; continue
        fi
        if awk -v g="$got" -v w="$want" 'BEGIN{d=g-w; if(d<0)d=-d; exit !(d<1e-6)}'; then
            echo "    $p = $got  (matches calibrated YAML)"
        else
            echo "    ERROR: $p = $got but calibrated YAML says $want"; ok=1
        fi
    done
    return $ok
}

run_one() {
    local dir="$1"
    echo ""
    echo ">>> Calibrated $dir run (pair $PAIR)"
    ros2 launch a2_description sim.launch.py pair:="$PAIR" own_odometry:=true \
        odometry_params_file:="$CALIB_YAML" >/dev/null 2>&1 &
    local sim_pid=$!

    if ! wait_for_topic /odom_own || ! wait_for_topic /ground_truth/odom; then
        echo "    ERROR: topics did not come up, skipping this run"
        kill "$sim_pid" 2>/dev/null || true; clean_sim; return 1
    fi
    sleep 2

    if ! verify_params; then
        echo "    Calibrated parameters were NOT loaded; skipping this run so no wrong data is recorded."
        kill "$sim_pid" 2>/dev/null || true; clean_sim; return 1
    fi

    ros2 run a2_description umbmark_driver.py --ros-args -p use_sim_time:=true \
        -p odom_topic:=/odom_own -p direction:="$dir" -p side:=2.0 -p csv:="$CSV_VAL" \
        || echo "    WARNING: driver returned an error"

    kill "$sim_pid" 2>/dev/null || true
    clean_sim
}

echo "=========================================================="
echo "      TASK 5(f): VALIDATION WITH CALIBRATED ODOMETRY"
echo "=========================================================="
clean_sim

for dir in cw ccw; do
    for _ in $(seq 1 "$N_RUNS"); do
        run_one "$dir" || true
    done
done

echo ""
echo "=========================================================="
echo "               TASK 5(f) VALIDATION RESULTS"
echo "=========================================================="
python3 "$REPO_ROOT/validation_report.py" --calibrated "$CSV_VAL" --baseline "$UMB_CSV"