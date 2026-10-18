#!/usr/bin/env bash
# run_scale_test.sh - Scale test for Task 5(e): c = d_true / d_odom = rbar / rn
# Usage: ./run_scale_test.sh [N_RUNS] [TARGET_METRES]
# Appends one row per run to scale_test.csv; prints mean and sd of c at the end.
set -u

N_RUNS="${1:-3}"
TARGET="${2:-2.0}"                # drive until odometry reports this distance
MAX_DRIVE_SECONDS=120             # wall-clock safety timeout
PAIR="P08"
CSV="scale_test.csv"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"
set +u                               # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/src_affan/install/setup.bash"
set -u

clean_sim() {
    for p in rviz2 "gz sim" gz-sim parameter_bridge robot_state_publisher \
             static_transform_publisher odometry_node; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
    sleep 2
}

# Prints "x y" of a nav_msgs/Odometry topic, or nothing on failure.
read_xy() {
    timeout 10 ros2 topic echo "$1" --once --field pose.pose.position 2>/dev/null \
      | awk '/^x:/{x=$2} /^y:/{y=$2} END{if (x != "" && y != "") print x, y}'
}

wait_for_topic() {
    for _ in $(seq 1 60); do
        if timeout 5 ros2 topic echo "$1" --once >/dev/null 2>&1; then return 0; fi
        sleep 1
    done
    return 1
}

# Wait until the robot has stopped moving (ground-truth position stops changing).
wait_settled() {
    local prev cur
    prev=$(read_xy /ground_truth/odom)
    for _ in $(seq 1 30); do
        sleep 1
        cur=$(read_xy /ground_truth/odom)
        if [ -n "$prev" ] && [ -n "$cur" ] && awk -v a="$prev" -v b="$cur" \
            'BEGIN{split(a,p," ");split(b,q," ");exit !(sqrt((q[1]-p[1])^2+(q[2]-p[2])^2)<0.0005)}'; then
            return 0
        fi
        prev=$cur
    done
    echo "    WARNING: robot did not settle within 30 s"
    return 1
}

echo "d_odom,d_true,c" > "$CSV"
clean_sim

for i in $(seq 1 "$N_RUNS"); do
    echo ""
    echo ">>> Scale test run $i of $N_RUNS (pair $PAIR)"
    ros2 launch a2_description sim.launch.py pair:="$PAIR" own_odometry:=true >/dev/null 2>&1 &
    SIM_PID=$!

    if ! wait_for_topic /odom_own || ! wait_for_topic /ground_truth/odom; then
        echo "ERROR: topics did not come up, skipping run $i"
        kill "$SIM_PID" 2>/dev/null || true; clean_sim; continue
    fi
    sleep 2

    START_O=$(read_xy /odom_own)
    START_T=$(read_xy /ground_truth/odom)

    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20}}" >/dev/null 2>&1 &
    PUB_PID=$!
    # Drive until odometry reports TARGET metres (robust to a slow simulator).
    DEADLINE=$((SECONDS + MAX_DRIVE_SECONDS))
    while [ "$SECONDS" -lt "$DEADLINE" ]; do
        CUR=$(read_xy /odom_own)
        if [ -n "$CUR" ] && [ -n "$START_O" ]; then
            DIST=$(awk -v a="$START_O" -v b="$CUR" \
                'BEGIN{split(a,s," ");split(b,c," ");dx=c[1]-s[1];dy=c[2]-s[2];print sqrt(dx*dx+dy*dy)}')
            if awk -v d="$DIST" -v t="$TARGET" 'BEGIN{exit !(d>=t)}'; then break; fi
        fi
    done
    kill "$PUB_PID" 2>/dev/null || true
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
    wait_settled

    END_O=$(read_xy /odom_own)
    END_T=$(read_xy /ground_truth/odom)

    if [ -z "$START_O" ] || [ -z "$START_T" ] || [ -z "$END_O" ] || [ -z "$END_T" ]; then
        echo "ERROR: could not read all four poses, skipping run $i"
    else
        python3 - "$START_O" "$END_O" "$START_T" "$END_T" "$CSV" <<'EOF'
import math, sys
so, eo, st, et, csv_path = sys.argv[1:6]
sx, sy = map(float, so.split()); ex, ey = map(float, eo.split())
tx, ty = map(float, st.split()); ux, uy = map(float, et.split())
d_odom = math.hypot(ex - sx, ey - sy)
d_true = math.hypot(ux - tx, uy - ty)
if d_odom < 0.5:
    sys.exit("ERROR: odometry distance too small ({:.3f} m), run discarded".format(d_odom))
c = d_true / d_odom
with open(csv_path, "a") as f:
    f.write("{:.6f},{:.6f},{:.6f}\n".format(d_odom, d_true, c))
print("    d_odom = {:.5f} m   d_true = {:.5f} m   c = {:.6f}".format(d_odom, d_true, c))
EOF
    fi

    kill "$SIM_PID" 2>/dev/null || true
    clean_sim
done

echo ""
python3 - "$CSV" <<'EOF'
import csv, statistics, sys
rows = list(csv.DictReader(open(sys.argv[1])))
cs = [float(r["c"]) for r in rows]
print("Valid runs: {}".format(len(cs)))
if cs:
    print("Mean c = {:.6f}".format(statistics.mean(cs)))
if len(cs) > 1:
    print("SD of c = {:.6f}".format(statistics.stdev(cs)))
EOF

[ -f "$REPO_ROOT/scale_calibrate.py" ] && python3 "$REPO_ROOT/scale_calibrate.py"