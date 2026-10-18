#!/usr/bin/env python3
"""
Analyzes CSV test runs, computes controller performance metrics,
and plots responses for Parts (b), (c), (d), and (e).
Note: AI assistance used for plotting code

Metrics computed:
  - Overshoot: How far D dips below setpoint D_ref = 0.5 m (in meters and %)
  - Settling Time: Time when robot stays within ±2 cm (0.48 - 0.52 m)
  - Steady-State Error: Mean offset (D - D_ref) over the final 5 seconds
  - Steering Noise (HF): Scan-to-scan turn rate chatter = std(diff(omega)) / sqrt(2)
"""

import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # headless plotting for scripts
import matplotlib.pyplot as plt
import numpy as np

# ==============================================================================
# 1. EXPERIMENT CONSTANTS & ARENA GEOMETRY
# ==============================================================================
D_REF = 0.5               # Setpoint distance to the wall (meters)
TOL = 0.02                # Settling tolerance band (±2 cm)
THETA_DEG = 45.0          # Angle separation between beam b (270°) and beam a (315°)
NOISE_WINDOW = 5.0        # Last N seconds of the run to measure steady-state
MAX_OMEGA = 1.5           # Actuator turn rate limit (rad/s)

# Pair P08 theoretical drift calculation for Part (d):
# r_L = 0.054146 m, r_R = 0.053854 m, b = 0.3190 m, v = 0.2 m/s
# omega_drift = v * (r_R - r_L) / (r_avg * b) = -0.00339 rad/s
# Steady state needs Kp*e = -omega_drift, so e = D_ref - D = +0.0034 m with Kp = 1.0,
# i.e. the robot settles 3.4 mm CLOSER to the wall: D - D_ref = -0.0034 m
PREDICTED_SS = -0.0034

WALL_Y = -6.00
DOOR_X_START = 2.00
DOOR_X_END = 3.00
RANGE_THRESHOLD = 1.5   


# ==============================================================================
# 2. HELPER FUNCTIONS
# ==============================================================================
def load_csv(csv_path):
    """Loads CSV file into a numpy structured array with named columns."""
    return np.genfromtxt(csv_path, delimiter=',', names=True)


def compute_metrics(time, D, omega):
    """Computes overshoot, settling time, steady-state error, and noise."""
    finite = np.isfinite(D)
    if not finite.any():
        return None

    t_end = float(time[-1])

    # 1. Overshoot (robot starts outside at D = 0.8 m, so it dips below D_ref = 0.5 m)
    d0 = D[finite][0]
    min_d = float(np.min(D[finite]))
    if min_d < D_REF:
        overshoot_m = D_REF - min_d
        overshoot_pct = (overshoot_m / (d0 - D_REF)) * 100.0 if d0 > D_REF else 0.0
    else:
        overshoot_m = 0.0
        overshoot_pct = 0.0

    # 2. Settling time within tolerance band [D_REF - TOL, D_REF + TOL]
    # Smooth D with a 5-point (0.5s) rolling average to filter out Gaussian LiDAR range noise (sigma_r = 1 cm)
    D_clean = np.where(finite, D, np.inf)
    if len(D_clean) >= 5:
        D_smooth = np.convolve(D_clean, np.ones(5)/5.0, mode='valid')
        time_smooth = time[2:-2]
    else:
        D_smooth = D_clean
        time_smooth = time

    MIN_SETTLE_DWELL = 2.5  # seconds of sustained tracking required
    in_band = np.abs(D_smooth - D_REF) <= TOL
    outside_indices = np.where(~in_band)[0]
    if len(outside_indices) == 0:
        settling_time = float(time_smooth[0])
    elif outside_indices[-1] == len(D_smooth) - 1:
        settling_time = float('nan')  # Did not settle
    else:
        candidate_time = float(time_smooth[outside_indices[-1] + 1])
        if (t_end - candidate_time) < MIN_SETTLE_DWELL:
            settling_time = float('nan')
        else:
            settling_time = candidate_time

    # 3. Steady-state error over the final NOISE_WINDOW seconds
    t_end = time[-1]
    final_window = finite & (time >= (t_end - NOISE_WINDOW))
    if final_window.any():
        ss_error = float(np.mean(D[final_window]) - D_REF)
        d_std = float(np.std(D[final_window]))
    else:
        ss_error = float('nan')
        d_std = float('nan')

    # 4. Turn rate noise (high-frequency chatter) over the final steady-state window
    omega_final_mask = np.isfinite(omega) & (time >= (t_end - NOISE_WINDOW))
    if np.sum(omega_final_mask) > 3:
        omega_vals = omega[omega_final_mask]
        omega_std = float(np.std(omega_vals))
        # Scan-to-scan difference removes intentional turn curves, isolates sensor noise chatter
        omega_noise = float(np.std(np.diff(omega_vals)) / math.sqrt(2.0))
    else:
        omega_std = float('nan')
        omega_noise = float('nan')

    return {
        'overshoot_m': overshoot_m,
        'overshoot_pct': overshoot_pct,
        'settling_time': settling_time,
        'ss_error': ss_error,
        'd_std': d_std,
        'omega_std': omega_std,
        'omega_noise': omega_noise,
    }


def print_metric_table(title, metrics_dict):
    """Prints a clean summary table of results."""
    print(f"\n--- {title} ---")
    print(f"{'Run / Controller':<18} | {'Overshoot [m]':>13} | {'Overshoot [%]':>13} | {'Settle Time [s]':>15} | {'SS Error [m]':>14} | {'Noise in w':>12}")
    print("-" * 94)
    for label, m in metrics_dict.items():
        st_str = f"{m['settling_time']:.2f}" if not math.isnan(m['settling_time']) else "did not settle"
        noise_str = f"{m['omega_noise']:.4f}" if not math.isnan(m['omega_noise']) else "N/A"
        print(f"{label:<18} | {m['overshoot_m']:>13.4f} | {m['overshoot_pct']:>13.1f} | {st_str:>15} | {m['ss_error']:>14.4f} | {noise_str:>12}")


# ==============================================================================
# 3. MAIN ANALYSIS & PLOTTING PIPELINE
# ==============================================================================
def main():
    repo_root = Path(__file__).resolve().parent
    runs_dir = repo_root / 'Wall Follower Runs'
    graphs_dir = repo_root / 'Graphs'
    graphs_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 75)
    print("PROBLEM 7: EXPERIMENTAL ANALYSIS & REPORT GENERATOR")
    print("=" * 75)

    # --------------------------------------------------------------------------
    # Part (b): P-Only Controller Comparison
    # --------------------------------------------------------------------------
    p_files = {
        'Kp = 0.5': runs_dir / 'p_kp0.5.csv',
        'Kp = 1.0': runs_dir / 'p_kp1.0.csv',
        'Kp = 1.8': runs_dir / 'p_kp1.8.csv',
    }
    p_data = {}
    p_metrics = {}
    for label, path in p_files.items():
        if path.is_file():
            data = load_csv(path)
            p_data[label] = data
            p_metrics[label] = compute_metrics(data['time'], data['D'], data['omega'])

    if p_data:
        print_metric_table("Part (b): P-Only Controller", p_metrics)

        # Plot Part (b)
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.axhline(D_REF, color='black', linestyle='--', linewidth=1.5, label=f'Setpoint (D_ref = {D_REF} m)')
        ax.axhspan(D_REF - TOL, D_REF + TOL, color='green', alpha=0.15, label='±2 cm tolerance band')

        colors = ['blue', 'orange', 'red']
        for (label, d), c in zip(p_data.items(), colors):
            ax.plot(d['time'], d['D'], label=label, color=c, linewidth=2.0)

        ax.set_title("Problem 7(b): P-Only Controller (Varying Kp)", fontsize=13, fontweight='bold')
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Perpendicular Distance D [m]")
        ax.set_xlim(0, max(d['time'][-1] for d in p_data.values()))
        ax.set_ylim(0.2, 0.85)
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', framealpha=0.9)
        plt.tight_layout()

        plt.savefig(graphs_dir / 'q7_b_p_only.png', dpi=300)
        plt.savefig(graphs_dir / 'q7_b_p_only.pdf')
        plt.close()
        print(f"[Saved] {graphs_dir / 'q7_b_p_only.png'}")

    # --------------------------------------------------------------------------
    # Part (c): PD Controller Comparison (Damping vs Steering Noise)
    # --------------------------------------------------------------------------
    pd_files = {
        'Kp=1.0, Kd=0.0': runs_dir / 'p_kp1.0.csv',
        'Kp=1.0, Kd=0.5': runs_dir / 'pd_kd0.5.csv',
        'Kp=1.0, Kd=1.0': runs_dir / 'pd_kd1.0.csv',
        'Kp=1.0, Kd=2.0': runs_dir / 'pd_kd2.0.csv',
    }
    pd_data = {}
    pd_metrics = {}
    for label, path in pd_files.items():
        if path.is_file():
            data = load_csv(path)
            pd_data[label] = data
            pd_metrics[label] = compute_metrics(data['time'], data['D'], data['omega'])

    # Only analyze and plot Part (c) when at least one actual PD run (Kd > 0) has been collected
    if len(pd_data) > 1:
        print_metric_table("Part (c): PD Controller", pd_metrics)

        # Plot Part (c) - 2 Subplots (Distance & Turn Rate)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.5, 7), sharex=True)
        ax1.axhline(D_REF, color='black', linestyle='--', linewidth=1.5, label=f'Setpoint ({D_REF} m)')
        ax1.axhspan(D_REF - TOL, D_REF + TOL, color='green', alpha=0.15, label='±2 cm band')

        pd_colors = ['gray', 'green', 'blue', 'red']
        for (label, d), c in zip(pd_data.items(), pd_colors):
            ax1.plot(d['time'], d['D'], label=label, color=c, linewidth=1.8)
            noise = pd_metrics[label]['omega_noise']
            tag = f"{label} (Noise={noise:.3f})" if not math.isnan(noise) else label
            ax2.plot(d['time'], d['omega'], label=tag, color=c, linewidth=1.2, alpha=0.85)

        ax1.set_title("Problem 7(c): PD Control - Distance Response", fontsize=12, fontweight='bold')
        ax1.set_ylabel("Distance D [m]")
        ax1.set_ylim(0.2, 0.85)
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend(loc='upper right', fontsize=9)

        ax2.set_title("Problem 7(c): Turn Rate (Derivative Noise Amplification)", fontsize=12, fontweight='bold')
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Turn Rate ω [rad/s]")
        ax2.set_xlim(0, max(d['time'][-1] for d in pd_data.values()))
        ax2.set_ylim(-MAX_OMEGA - 0.1, MAX_OMEGA + 0.1)
        ax2.grid(True, linestyle=':', alpha=0.6)
        ax2.legend(loc='upper right', fontsize=9)
        plt.tight_layout()

        plt.savefig(graphs_dir / 'q7_c_pd.png', dpi=300)
        plt.savefig(graphs_dir / 'q7_c_pd.pdf')
        plt.close()
        print(f"[Saved] {graphs_dir / 'q7_c_pd.png'}")

    # --------------------------------------------------------------------------
    # Part (d): Steady-State Offset & Is Integral Term Needed?
    # --------------------------------------------------------------------------
    print("\n--- Part (d): Steady-State Analysis & Integral Term ---")
    best = pd_data.get('Kp=1.0, Kd=2.0')
    if best is not None:
        t_b, D_b = best['time'], best['D']
        m = np.isfinite(D_b) & (t_b >= 15.0)
        x = D_b[m] - D_REF
        print(f"Best PD (Kp=1.0, Kd=2.0), t >= 15 s: mean offset {x.mean()*1000:+.1f} mm "
              f"(std {x.std()*1000:.1f} mm, s.e.m. {x.std()/math.sqrt(len(x))*1000:.1f} mm)")
    print(f"Predicted offset from wheel mismatch (pair P08): {PREDICTED_SS*1000:+.1f} mm")
    print("Both are well below the +/-20 mm band and the ~10 mm per-scan LiDAR noise,")
    print("so an Integral term is not worth having (windup risk at corners/doorways).")

    # --------------------------------------------------------------------------
    # Part (e): Doorway Gap Failure Analysis
    # --------------------------------------------------------------------------
    doorway_path = runs_dir / 'doorway_failure.csv'
    if doorway_path.is_file():
        d = load_csv(doorway_path)
        t, a, b = d['time'], d['a_range'], d['b_range']
        print("\n--- Part (e): Doorway Failure Analysis ---")

        # Beam detection of the doorway gap
        a_sees_gap = ~np.isfinite(a) | (a > RANGE_THRESHOLD)
        b_sees_gap = ~np.isfinite(b) | (b > RANGE_THRESHOLD)
        ia = int(np.flatnonzero(a_sees_gap)[0]) if a_sees_gap.any() else None
        ib = int(np.flatnonzero(b_sees_gap)[0]) if b_sees_gap.any() else None

        if ia is not None:
            print(f"Beam a (315°) first detects doorway at t = {t[ia]:.2f} s (robot x = {d['gt_x'][ia]:.2f} m)")
        if ib is not None:
            print(f"Beam b (270°) detects doorway later at   t = {t[ib]:.2f} s (robot x = {d['gt_x'][ib]:.2f} m)")
        if ia is not None and ib is not None:
            dx_measured = d['gt_x'][ib] - d['gt_x'][ia]
            dx_theory = D_REF * math.tan(math.radians(THETA_DEG))
            print(f"Advance detection distance: {dx_measured:.2f} m (Theory: D_ref * tan(45°) = {dx_theory:.2f} m)")

        # Plot Part (e) - Trajectory, LiDAR Ranges, and Turn Rate
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8.5, 9.5))

        # 1. 2D Trajectory vs Arena South Wall & Doorway
        ax1.plot(d['gt_x'], d['gt_y'], 'b-', linewidth=2.0, label='Robot Trajectory')
        # Wall segment 1: x in [-6.0, 2.0]
        ax1.plot([-6.0, DOOR_X_START], [WALL_Y, WALL_Y], 'k-', linewidth=4.0, label='Solid Wall')
        # Doorway gap: x in [2.0, 3.0]
        ax1.plot([DOOR_X_START, DOOR_X_END], [WALL_Y, WALL_Y], 'r--', linewidth=2.5, label='Doorway Gap (1 m)')
        # Wall segment 2: x in [3.0, 6.0]
        ax1.plot([DOOR_X_END, 6.0], [WALL_Y, WALL_Y], 'k-', linewidth=4.0)
        ax1.set_title("Problem 7(e): Trajectory Turning into the Doorway", fontsize=12, fontweight='bold')
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")
        ax1.set_xlim(-4.5, 4.0)
        ax1.set_ylim(-6.5, -4.8)
        ax1.set_aspect('equal', adjustable='datalim')
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend(loc='best', fontsize=9)

        # 2. LiDAR ranges for beams a and b
        a_clipped = np.clip(np.where(np.isfinite(a), a, 8.0), 0, 8.0)
        b_clipped = np.clip(np.where(np.isfinite(b), b, 8.0), 0, 8.0)
        ax2.plot(t, a_clipped, 'r-', linewidth=1.8, label='Beam a (315°)')
        ax2.plot(t, b_clipped, 'b--', linewidth=1.8, label='Beam b (270°)')
        ax2.set_title("LiDAR Beam Readings (Capped at 8 m)", fontsize=12, fontweight='bold')
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Range [m]")
        ax2.set_ylim(0, 8.5)
        ax2.grid(True, linestyle=':', alpha=0.6)
        ax2.legend(loc='upper left')

        # 3. Commanded turn rate
        ax3.plot(t, d['omega'], 'g-', linewidth=1.5)
        ax3.set_title("Commanded Turn Rate ω (Saturating into Gap)", fontsize=12, fontweight='bold')
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("ω [rad/s]")
        ax3.set_ylim(-MAX_OMEGA - 0.1, MAX_OMEGA + 0.1)
        ax3.grid(True, linestyle=':', alpha=0.6)

        plt.tight_layout()
        plt.savefig(graphs_dir / 'q7_e_doorway.png', dpi=300)
        plt.savefig(graphs_dir / 'q7_e_doorway.pdf')
        plt.close()
        print(f"[Saved] {graphs_dir / 'q7_e_doorway.png'}")

    print("=" * 75)
    print("ANALYSIS FINISHED")
    print("=" * 75)


if __name__ == '__main__':
    main()