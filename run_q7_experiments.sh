#!/usr/bin/env bash
# run_q7_experiments.sh - Automated runner for Problem 7 experiments
#
# Usage:  ./run_q7_experiments.sh [all|b|c|d|e]
#   b : P-only sweep          c : PD sweep (Kp fixed, vary Kd)
#   d : one PID run (optional, not part of "all")
#   e : doorway failure run   (HELD_RUN=1 adds a second run with hold_invalid:=true)
#
# Environment overrides (all optional):
#   PAIR=P08   GUI=true|false   BEST_KP=1.0   BEST_KD=1.0   KI=0.2
#   HELD_RUN=1   NO_ANALYSIS=1
# GUI=false only works if sim.launch.py accepts gui:=false; check the launch file.
set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/src_affan/install/setup.bash"
set -u

PAIR="${PAIR:-P08}"
GUI="${GUI:-true}"
BEST_KP="${BEST_KP:-1.0}"
BEST_KD="${BEST_KD:-1.0}"
KI="${KI:-0.2}"
RUNS_DIR="$REPO_ROOT/Wall Follower Runs"
mkdir -p "$RUNS_DIR"

clean_sim() {
    pkill -9 -f rviz2 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gz-sim" 2>/dev/null || true
    pkill -9 -f parameter_bridge 2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f static_transform_publisher 2>/dev/null || true
    pkill -9 -f "wall_follower" 2>/dev/null || true
    sleep 2
}

# Wait until /scan actually delivers a message (instead of a fixed sleep).
wait_for_scan() {
    local tries=0
    until timeout 5 ros2 topic echo --once --qos-reliability best_effort /scan >/dev/null 2>&1; do
        tries=$((tries + 1))
        if [ "$tries" -ge 12 ]; then
            echo "    [ERROR] /scan never delivered a message (see /tmp/sim_q7.log)"
            return 1
        fi
        sleep 2
    done
    sleep 2   # let the sim clock and ground truth settle
    return 0
}

# run_test KP KD KI DURATION CSV LABEL [extra ros args, e.g. -p hold_invalid:=true]
run_test() {
    local kp="$1" kd="$2" ki="$3" dur="$4" csv_file="$5" label="$6"
    shift 6
    local extra=("$@")

    echo ""
    echo "======================================================================"
    echo ">>> Running test: $label"
    echo "    Kp=$kp, Kd=$kd, Ki=$ki, Duration=${dur}s, pair=$PAIR, gui=$GUI"
    echo "    Target CSV: $csv_file"
    echo "======================================================================"

    clean_sim

    # Launch simulation at the Problem 6/7 start pose
    ros2 launch a2_description sim.launch.py \
        pair:="$PAIR" \
        gui:="$GUI" \
        x:=-4.0 y:=-5.2 >/tmp/sim_q7.log 2>&1 &
    SIM_PID=$!

    echo "    Waiting for /scan..."
    if wait_for_scan; then
        local limit=$(( ${dur%.*} + 60 ))
        timeout -s INT "$limit" ros2 run a2_description wall_follower.py --ros-args \
            -p use_sim_time:=true \
            -p Kp:="$kp" \
            -p Kd:="$kd" \
            -p Ki:="$ki" \
            -p duration:="$dur" \
            -p csv_path:="$csv_file" \
            ${extra[@]+"${extra[@]}"} || true

        if [ -f "$csv_file" ]; then
            echo "    Logged $(( $(wc -l < "$csv_file") - 1 )) rows"
        else
            echo "    [WARNING] no CSV was written"
        fi
    else
        echo "    [SKIPPED] simulation did not come up"
    fi

    kill -9 "$SIM_PID" 2>/dev/null || true
    clean_sim
    echo ">>> Finished: $label"
}

# Make sure simulation processes are cleaned up on exit
trap clean_sim EXIT INT TERM

TARGET_PART="${1:-all}"

# -------------------------------------------------------------
# Problem 7(b): P-only (Kd = Ki = 0, vary Kp)
# -------------------------------------------------------------
if [ "$TARGET_PART" = "all" ] || [ "$TARGET_PART" = "b" ]; then
    echo ""
    echo "######## Part (b): P-only ########"
    run_test "0.5" "0.0" "0.0" "25.0" "$RUNS_DIR/p_kp0.5.csv" "P-only (Kp = 0.5)"
    run_test "1.0" "0.0" "0.0" "25.0" "$RUNS_DIR/p_kp1.0.csv" "P-only (Kp = 1.0)"
    run_test "1.8" "0.0" "0.0" "25.0" "$RUNS_DIR/p_kp1.8.csv" "P-only (Kp = 1.8)"
fi

# -------------------------------------------------------------
# Problem 7(c): PD (Kp = 1.0, vary Kd)
# -------------------------------------------------------------
if [ "$TARGET_PART" = "all" ] || [ "$TARGET_PART" = "c" ]; then
    echo ""
    echo "######## Part (c): PD ########"
    run_test "1.0" "0.5" "0.0" "25.0" "$RUNS_DIR/pd_kd0.5.csv" "PD (Kp = 1.0, Kd = 0.5)"
    run_test "1.0" "1.0" "0.0" "25.0" "$RUNS_DIR/pd_kd1.0.csv" "PD (Kp = 1.0, Kd = 1.0)"
    run_test "1.0" "2.0" "0.0" "25.0" "$RUNS_DIR/pd_kd2.0.csv" "PD (Kp = 1.0, Kd = 2.0)"
fi

# -------------------------------------------------------------
# Problem 7(d): optional PID run with your chosen gains (not in "all")
# -------------------------------------------------------------
if [ "$TARGET_PART" = "d" ]; then
    echo ""
    echo "######## Part (d): PID (optional) ########"
    run_test "$BEST_KP" "$BEST_KD" "$KI" "40.0" "$RUNS_DIR/pid_ki.csv" \
        "PID (Kp = $BEST_KP, Kd = $BEST_KD, Ki = $KI)"
fi

# -------------------------------------------------------------
# Problem 7(e): doorway failure (about 30 s to reach the doorway at 0.2 m/s)
# -------------------------------------------------------------
if [ "$TARGET_PART" = "all" ] || [ "$TARGET_PART" = "e" ]; then
    echo ""
    echo "######## Part (e): doorway failure ########"
    run_test "$BEST_KP" "$BEST_KD" "0.0" "100.0" "$RUNS_DIR/doorway_failure.csv" "Doorway failure (raw)"
    if [ "${HELD_RUN:-0}" = "1" ]; then
        run_test "$BEST_KP" "$BEST_KD" "0.0" "60.0" "$RUNS_DIR/doorway_held.csv" \
            "Doorway failure (hold_invalid)" -p hold_invalid:=true
    fi
fi

# -------------------------------------------------------------
# Analysis and plots
# -------------------------------------------------------------
if [ "${NO_ANALYSIS:-0}" != "1" ]; then
    echo ""
    echo "######## Processing data and generating graphs ########"
    python3 analyze_q7.py
    echo ""
    echo "Finished. Graphs and metrics are in Graphs/."
fi