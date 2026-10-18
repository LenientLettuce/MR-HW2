#!/usr/bin/env python3
"""plot_umbmark.py - Analyze UMBmark experiment data, calculate CGs, and plot endpoints.

Usage:
    python3 Graphs/plot_umbmark.py [--csv umbmark.csv] [--side 2.0] [--method "RK2 (50 Hz)"]
"""

import argparse
import csv
import math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description='Analyze UMBmark experiment data.')
    parser.add_argument('--csv', default='umbmark.csv', help='Path to umbmark.csv')
    parser.add_argument('--side', type=float, default=2.0, help='Square side length in metres (default 2.0)')
    parser.add_argument('--rn', type=float, default=0.053, help='Nameplate wheel radius in metres (P08: 0.053)')
    parser.add_argument('--bn', type=float, default=0.313, help='Nameplate track width in metres (P08: 0.313)')
    parser.add_argument('--r-left-actual', type=float, default=0.054146, help='Actual physical r_left (P08: 0.054146)')
    parser.add_argument('--r-right-actual', type=float, default=0.053854, help='Actual physical r_right (P08: 0.053854)')
    parser.add_argument('--b-actual', type=float, default=0.3190, help='Actual physical track width b (P08: 0.3190)')
    parser.add_argument('--method', default='unspecified', help='Integration method (default: unspecified)')
    parser.add_argument('--output-dir', default='Graphs', help='Output directory for plots and report')
    return parser.parse_args()


def load_umbmark_data(csv_path):
    cw_runs = []
    ccw_runs = []
    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            d = row['direction'].strip().lower()
            ex = float(row['eps_x'])
            ey = float(row['eps_y'])
            eth = float(row['eps_theta'])
            if d == 'cw':
                cw_runs.append((ex, ey, eth))
            elif d == 'ccw':
                ccw_runs.append((ex, ey, eth))
    return np.array(cw_runs), np.array(ccw_runs)


def main():
    args = parse_args()
    csv_file = Path(args.csv)
    if not csv_file.exists():
        print(f"Error: {csv_file} does not exist.")
        return

    cw, ccw = load_umbmark_data(csv_file)
    if len(cw) == 0 or len(ccw) == 0:
        print(f"Error: Need both CW and CCW runs. Found {len(cw)} CW runs and {len(ccw)} CCW runs.")
        return

    # 7. Check if number of CW or CCW runs is not 5
    if len(cw) != 5:
        print(f"Warning: Expected 5 CW runs, but found {len(cw)}.")
    if len(ccw) != 5:
        print(f"Warning: Expected 5 CCW runs, but found {len(ccw)}.")

    L = args.side
    rn = args.rn
    bn = args.bn

    # Actual physical parameters for pair P08
    r_L_act = args.r_left_actual
    r_R_act = args.r_right_actual
    r_bar_act = (r_L_act + r_R_act) / 2.0
    b_act = args.b_actual

    # Theoretical systematic errors from Task 5(a)
    alpha_pred = (math.pi / 2.0) * ((r_bar_act * bn) / (rn * b_act) - 1.0)
    beta_pred = (L * (r_R_act - r_L_act)) / (rn * b_act)

    # Theoretical predicted CGs from Task 5(b)
    xg_cw_pred = 2.0 * L * (alpha_pred - beta_pred)
    yg_cw_pred = 2.0 * L * (alpha_pred - beta_pred)

    xg_ccw_pred = 2.0 * L * (alpha_pred + beta_pred)
    yg_ccw_pred = -2.0 * L * (alpha_pred + beta_pred)

    # Measured Centres of gravity (average of trials)
    xg_cw = np.mean(cw[:, 0])
    yg_cw = np.mean(cw[:, 1])
    th_cw = np.mean(cw[:, 2])

    xg_ccw = np.mean(ccw[:, 0])
    yg_ccw = np.mean(ccw[:, 1])
    th_ccw = np.mean(ccw[:, 2])

    # Solving for measured alpha and beta using both eps_x and eps_y:
    # CCW: eps_x ≈ 2L(alpha + beta), eps_y ≈ -2L(alpha + beta)  -> (alpha + beta) ≈ (eps_x - eps_y) / (4L)
    # CW:  eps_x ≈ 2L(alpha - beta), eps_y ≈  2L(alpha - beta)  -> (alpha - beta) ≈ (eps_x + eps_y) / (4L)
    sum_ab = (xg_ccw - yg_ccw) / (4.0 * L)
    diff_ab = (xg_cw + yg_cw) / (4.0 * L)

    alpha_meas = (sum_ab + diff_ab) / 2.0
    beta_meas = (sum_ab - diff_ab) / 2.0

    # Corrected parameters from UMBmark
    # 1. Track width b'
    b_prime = bn * (math.pi / 2.0) / (math.pi / 2.0 + alpha_meas)

    # 2. Wheel diameter ratio Ed = rR / rL
    # beta = L*(rR - rL)/(rn*b), and the corrected width b_prime is used because rn*b = b_prime*rbar
    term = beta_meas * b_prime / L
    Ed = (2.0 + term) / (2.0 - term)

    # Radii assuming average is rn for now
    r_left_umb = 2.0 * rn / (1.0 + Ed)
    r_right_umb = Ed * r_left_umb

    # 2. Computed comparison variables from data
    beta_err_pct = abs(beta_meas - beta_pred) / abs(beta_pred) * 100
    Ed_true = r_R_act / r_L_act
    Ed_err_pct = abs(Ed - Ed_true) / Ed_true * 100
    dr_err_mm = abs((r_left_umb - r_right_umb) - (r_L_act - r_R_act)) * 1e3
    spread_cw_mm = np.max(np.sqrt((cw[:, 0] - xg_cw)**2 + (cw[:, 1] - yg_cw)**2)) * 1e3
    spread_ccw_mm = np.max(np.sqrt((ccw[:, 0] - xg_ccw)**2 + (ccw[:, 1] - yg_ccw)**2)) * 1e3
    max_disc_cm = max(abs(xg_cw - xg_cw_pred), abs(yg_cw - yg_cw_pred),
                      abs(xg_ccw - xg_ccw_pred), abs(yg_ccw - yg_ccw_pred)) * 100

    # Console summary formatted as ready-to-screenshot tables
    sep = "=" * 96
    sub_sep = "-" * 96
    print("\n" + sep)
    print("           PROBLEM 5: UMBMARK CALIBRATION & PARAMETER IDENTIFICATION (TASKS 5c & 5d)")
    print(sep)
    print(f"Robot: Pair P08 | Leg Length L = {L:.3f} m | Integration: {args.method}")
    print(f"Nameplate: rn = {rn:.6f} m, bn = {bn:.6f} m")
    print(f"Actual   : r_left = {r_L_act:.6f} m, r_right = {r_R_act:.6f} m, b = {b_act:.6f} m")
    print(sub_sep)

    print("\n[TASK 5c] TABLE 1: RAW EXPERIMENTAL UMBMARK TRIALS (5 CW, 5 CCW)")
    print("+----------+-----------+--------------------+--------------------+--------------------+")
    print("| Run #    | Direction | eps_x Error (cm)   | eps_y Error (cm)   | eps_th Error (deg) |")
    print("+----------+-----------+--------------------+--------------------+--------------------+")
    for i, row in enumerate(cw):
        print(f"| Trial {i+1:<2} | CW        |      {row[0]*100:+7.2f}       |      {row[1]*100:+7.2f}       |      {math.degrees(row[2]):+7.2f}°     |")
    print("+----------+-----------+--------------------+--------------------+--------------------+")
    print(f"| CW c.g.  | CW MEAN   |      {xg_cw*100:+7.2f}       |      {yg_cw*100:+7.2f}       |      {math.degrees(th_cw):+7.2f}°     |")
    print("+----------+-----------+--------------------+--------------------+--------------------+")
    for i, row in enumerate(ccw):
        print(f"| Trial {i+1:<2} | CCW       |      {row[0]*100:+7.2f}       |      {row[1]*100:+7.2f}       |      {math.degrees(row[2]):+7.2f}°     |")
    print("+----------+-----------+--------------------+--------------------+--------------------+")
    print(f"| CCW c.g. | CCW MEAN  |      {xg_ccw*100:+7.2f}       |      {yg_ccw*100:+7.2f}       |      {math.degrees(th_ccw):+7.2f}°     |")
    print("+----------+-----------+--------------------+--------------------+--------------------+")

    print("\n[TASK 5c] TABLE 2: CENTRES OF GRAVITY - MEASURED VS FIRST-ORDER PREDICTION")
    print("+-----------+----------------------------+----------------------------+----------------------------+")
    print("| Direction | Predicted CG (eps_x, y)    | Measured CG (eps_x, y)     | Discrepancy (dx, dy)       |")
    print("+-----------+----------------------------+----------------------------+----------------------------+")
    print(f"| CW        | ({xg_cw_pred*100:+6.2f} cm, {yg_cw_pred*100:+6.2f} cm)   | ({xg_cw*100:+6.2f} cm, {yg_cw*100:+6.2f} cm)   | ({(xg_cw - xg_cw_pred)*100:+6.2f} cm, {(yg_cw - yg_cw_pred)*100:+6.2f} cm)   |")
    print(f"| CCW       | ({xg_ccw_pred*100:+6.2f} cm, {yg_ccw_pred*100:+6.2f} cm)   | ({xg_ccw*100:+6.2f} cm, {yg_ccw*100:+6.2f} cm)   | ({(xg_ccw - xg_ccw_pred)*100:+6.2f} cm, {(yg_ccw - yg_ccw_pred)*100:+6.2f} cm)   |")
    print("+-----------+----------------------------+----------------------------+----------------------------+")

    print("\n[TASK 5d] TABLE 3: SYSTEMATIC ERROR IDENTIFICATION (USING BOTH eps_x AND eps_y)")
    print(f"Formulas: (alpha + beta) = (eps_x,ccw - eps_y,ccw) / (4L) = {sum_ab:+.6f} rad")
    print(f"          (alpha - beta) = (eps_x,cw + eps_y,cw)   / (4L) = {diff_ab:+.6f} rad")
    print("+--------------------------+-----------------------+-----------------------+-----------------------+")
    print("| Systematic Error         | Theoretical Pred      | Identified (UMBmark)  | Difference / Accuracy |")
    print("+--------------------------+-----------------------+-----------------------+-----------------------+")
    print(f"| Turn Error (alpha)       | {alpha_pred:+9.6f} rad       | {alpha_meas:+9.6f} rad       | {math.degrees(alpha_meas - alpha_pred):+6.4f}° offset        |")
    print(f"|                          | ({math.degrees(alpha_pred):+7.4f}°)            | ({math.degrees(alpha_meas):+7.4f}°)            |                       |")
    print(f"| Curvature Error (beta)   | {beta_pred:+9.6f} rad       | {beta_meas:+9.6f} rad       | {beta_err_pct:.2f}% error            |")
    print(f"|                          | ({math.degrees(beta_pred):+7.4f}°)            | ({math.degrees(beta_meas):+7.4f}°)            |                       |")
    print("+--------------------------+-----------------------+-----------------------+-----------------------+")

    print("\n[TASK 5d] TABLE 4: CALIBRATED PARAMETERS (BEFORE SCALE TEST, HOLDING rn = 0.053 m)")
    print("+--------------------------+--------------------+--------------------+--------------------+--------------------+")
    print("| Parameter                | Nameplate (Init)   | Calibrated (5d)    | Physical Truth     | Status / Accuracy  |")
    print("+--------------------------+--------------------+--------------------+--------------------+--------------------+")
    print(f"| Track width (b)          | {bn:14.6f} m   | {b_prime:14.6f} m   | {b_act:14.6f} m   | b' = b*(rn/rbar)   |")
    print(f"| Wheel ratio Ed (rR/rL)   | {1.0:14.6f}     | {Ed:14.6f}     | {Ed_true:14.6f}     | {Ed_err_pct:.3f}% error      |")
    print(f"| Left wheel radius (rL)   | {rn:14.6f} m   | {r_left_umb:14.6f} m   | {r_L_act:14.6f} m   | Unscaled (5e next) |")
    print(f"| Right wheel radius (rR)  | {rn:14.6f} m   | {r_right_umb:14.6f} m   | {r_R_act:14.6f} m   | Unscaled (5e next) |")
    print(f"| Difference (rL - rR)     | {0.0:12.3f} mm   | {(r_left_umb - r_right_umb)*1e3:12.3f} mm   | {(r_L_act - r_R_act)*1e3:12.3f} mm   | {dr_err_mm:.3f} mm error   |")
    print("+--------------------------+--------------------+--------------------+--------------------+--------------------+")
    print(sep + "\n")

    # Plotting with Insets
    fig, ax = plt.subplots(figsize=(10, 9))

    # Origin (Ideal return pose)
    ax.scatter([0], [0], color='green', marker='P', s=180, edgecolors='black', zorder=5,
               label='Ideal Return: (0.00, 0.00) cm')

    # Individual trial endpoints (scatter)
    ax.scatter(cw[:, 0] * 100, cw[:, 1] * 100, color='tomato', marker='s', s=70, alpha=0.7,
               edgecolors='darkred', zorder=4, label=f'CW Trials (n={len(cw)})')
    ax.scatter(ccw[:, 0] * 100, ccw[:, 1] * 100, color='dodgerblue', marker='o', s=70, alpha=0.7,
               edgecolors='darkblue', zorder=4, label=f'CCW Trials (n={len(ccw)})')

    # Measured Centres of Gravity
    ax.scatter([xg_cw * 100], [yg_cw * 100], color='red', marker='X', s=200, edgecolors='black',
               linewidths=1.5, zorder=6, label=f'CW Measured CG: ({xg_cw*100:+.2f}, {yg_cw*100:+.2f}) cm')
    ax.scatter([xg_ccw * 100], [yg_ccw * 100], color='blue', marker='X', s=200, edgecolors='black',
               linewidths=1.5, zorder=6, label=f'CCW Measured CG: ({xg_ccw*100:+.2f}, {yg_ccw*100:+.2f}) cm')

    # Theoretical Predicted Centres of Gravity
    ax.scatter([xg_cw_pred * 100], [yg_cw_pred * 100], facecolors='none', edgecolors='crimson',
               marker='D', s=160, linewidths=2.2, linestyle='--', zorder=6,
               label=f'CW Predicted CG: ({xg_cw_pred*100:+.2f}, {yg_cw_pred*100:+.2f}) cm')
    ax.scatter([xg_ccw_pred * 100], [yg_ccw_pred * 100], facecolors='none', edgecolors='navy',
               marker='D', s=160, linewidths=2.2, linestyle='--', zorder=6,
               label=f'CCW Predicted CG: ({xg_ccw_pred*100:+.2f}, {yg_ccw_pred*100:+.2f}) cm')

    # Connector lines from predicted to measured
    ax.plot([xg_cw_pred * 100, xg_cw * 100], [yg_cw_pred * 100, yg_cw * 100], 'r:', linewidth=1.8, zorder=3)
    ax.plot([xg_ccw_pred * 100, xg_ccw * 100], [yg_ccw_pred * 100, yg_ccw * 100], 'b:', linewidth=1.8, zorder=3)

    # Reference axes
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.7)
    ax.axvline(0, color='gray', linestyle='--', linewidth=0.8, alpha=0.7)

    # Inset for CW Trials (Zoomed in, cm)
    # [x0, y0, width, height] in axes coordinates
    axins_cw = ax.inset_axes([0.55, 0.28, 0.40, 0.28])
    axins_cw.scatter(cw[:, 0] * 100, cw[:, 1] * 100, color='tomato', marker='s', s=90, edgecolors='darkred')
    for i, (pt_x, pt_y) in enumerate(zip(cw[:, 0] * 100, cw[:, 1] * 100)):
        axins_cw.annotate(f"#{i+1}", (pt_x, pt_y), textcoords="offset points", xytext=(5, 5), fontsize=8)
    axins_cw.scatter([xg_cw * 100], [yg_cw * 100], color='red', marker='X', s=140, edgecolors='black', label='CW CG')
    cw_x_min, cw_x_max = np.min(cw[:, 0] * 100) - 0.15, np.max(cw[:, 0] * 100) + 0.15
    cw_y_min, cw_y_max = np.min(cw[:, 1] * 100) - 0.15, np.max(cw[:, 1] * 100) + 0.15
    axins_cw.set_xlim(cw_x_min, cw_x_max)
    axins_cw.set_ylim(cw_y_min, cw_y_max)
    axins_cw.set_title("CW Cluster Inset (zoomed, cm)", fontsize=9, fontweight='bold', pad=4)
    axins_cw.grid(True, linestyle=':', alpha=0.5)
    ax.indicate_inset_zoom(axins_cw, edgecolor="tomato", alpha=0.8)

    # Inset for CCW Trials (Zoomed in, cm)
    axins_ccw = ax.inset_axes([0.05, 0.28, 0.40, 0.28])
    axins_ccw.scatter(ccw[:, 0] * 100, ccw[:, 1] * 100, color='dodgerblue', marker='o', s=90, edgecolors='darkblue')
    for i, (pt_x, pt_y) in enumerate(zip(ccw[:, 0] * 100, ccw[:, 1] * 100)):
        axins_ccw.annotate(f"#{i+1}", (pt_x, pt_y), textcoords="offset points", xytext=(5, 5), fontsize=8)
    axins_ccw.scatter([xg_ccw * 100], [yg_ccw * 100], color='blue', marker='X', s=140, edgecolors='black', label='CCW CG')
    ccw_x_min, ccw_x_max = np.min(ccw[:, 0] * 100) - 0.20, np.max(ccw[:, 0] * 100) + 0.20
    ccw_y_min, ccw_y_max = np.min(ccw[:, 1] * 100) - 0.20, np.max(ccw[:, 1] * 100) + 0.20
    axins_ccw.set_xlim(ccw_x_min, ccw_x_max)
    axins_ccw.set_ylim(ccw_y_min, ccw_y_max)
    axins_ccw.set_title("CCW Cluster Inset (zoomed, cm)", fontsize=9, fontweight='bold', pad=4)
    axins_ccw.grid(True, linestyle=':', alpha=0.5)
    ax.indicate_inset_zoom(axins_ccw, edgecolor="dodgerblue", alpha=0.8)

    ax.set_title("UMBmark Return Position Errors: Measured vs Predicted (Task 5c)", fontsize=13, pad=12, fontweight='bold')
    ax.set_xlabel("$\\varepsilon_x$ Return Error (cm)", fontsize=11)
    ax.set_ylabel("$\\varepsilon_y$ Return Error (cm)", fontsize=11)
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.22), ncol=2, framealpha=0.95, fontsize=9.5)
    ax.grid(True, linestyle=':', alpha=0.6)

    # 6. Axis limits computed from data with 12% margin
    all_x = np.concatenate([[0.0, xg_cw * 100, xg_ccw * 100, xg_cw_pred * 100, xg_ccw_pred * 100], cw[:, 0] * 100, ccw[:, 0] * 100])
    all_y = np.concatenate([[0.0, yg_cw * 100, yg_ccw * 100, yg_cw_pred * 100, yg_ccw_pred * 100], cw[:, 1] * 100, ccw[:, 1] * 100])
    min_x, max_x = float(np.min(all_x)), float(np.max(all_x))
    min_y, max_y = float(np.min(all_y)), float(np.max(all_y))
    span_x = max_x - min_x
    span_y = max_y - min_y
    margin_x = 0.12 * span_x if span_x > 0 else 1.0
    margin_y = 0.12 * span_y if span_y > 0 else 1.0
    ax.set_xlim(min_x - margin_x, max_x + margin_x)
    ax.set_ylim(min_y - margin_y, max_y + margin_y)
    ax.set_aspect('equal', 'datalim')

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    png_path = out_dir / 'umbmark_plot.png'
    pdf_path = out_dir / 'umbmark_plot.pdf'
    plt.tight_layout()
    plt.savefig(png_path, dpi=300)
    plt.savefig(pdf_path)
    print(f"Saved plots to {png_path} and {pdf_path}")

    # Write Comprehensive Markdown Report
    report_path = out_dir / 'umbmark_report.md'
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write("# Problem 5: UMBmark Calibration & Parameter Identification Report\n\n")
        f.write("## Experimental Setup\n")
        f.write(f"- **Robot Assignment:** Pair P08\n")
        f.write(f"- **Square Side Length ($L$):** {L:.3f} m\n")
        f.write(f"- **Odometry Integration:** {args.method}\n")
        f.write(f"- **Nameplate Parameters (Uncalibrated):** $r_n = {rn:.6f}\\text{{ m}}, \\quad b_n = {bn:.6f}\\text{{ m}}$\n")
        f.write(f"- **Actual Physical Parameters (Gazebo Truth):** $r_L = {r_L_act:.6f}\\text{{ m}}, \\quad r_R = {r_R_act:.6f}\\text{{ m}}, \\quad b = {b_act:.6f}\\text{{ m}}$\n\n")

        f.write("## [Task 5c] Table 1: Raw Experimental UMBmark Trial Data\n\n")
        f.write("| Run # | Direction | $\\varepsilon_x$ Error (cm) | $\\varepsilon_y$ Error (cm) | $\\varepsilon_\\theta$ Error (deg) |\n")
        f.write("| :---: | :---: | :---: | :---: | :---: |\n")
        for i, row in enumerate(cw):
            f.write(f"| Trial {i+1} | CW | ${row[0]*100:+.2f}$ | ${row[1]*100:+.2f}$ | ${math.degrees(row[2]):+.2f}^\\circ$ |\n")
        f.write(f"| **CW c.g.** | **CW Mean** | **${xg_cw*100:+.2f}$** | **${yg_cw*100:+.2f}$** | **${math.degrees(th_cw):+.2f}^\\circ$** |\n")
        for i, row in enumerate(ccw):
            f.write(f"| Trial {i+1} | CCW | ${row[0]*100:+.2f}$ | ${row[1]*100:+.2f}$ | ${math.degrees(row[2]):+.2f}^\\circ$ |\n")
        f.write(f"| **CCW c.g.** | **CCW Mean** | **${xg_ccw*100:+.2f}$** | **${yg_ccw*100:+.2f}$** | **${math.degrees(th_ccw):+.2f}^\\circ$** |\n\n")

        f.write("## [Task 5c] Table 2: Centres of Gravity - Measured vs First-Order Prediction\n\n")
        f.write("| Direction | Predicted CG $(\\varepsilon_x, \\varepsilon_y)$ | Measured CG $(\\varepsilon_x, \\varepsilon_y)$ | Discrepancy $(\\Delta \\varepsilon_x, \\Delta \\varepsilon_y)$ |\n")
        f.write("| :--- | :---: | :---: | :---: |\n")
        f.write(f"| **CW** | $({xg_cw_pred*100:+.2f}, {yg_cw_pred*100:+.2f})\\text{{ cm}}$ | $({xg_cw*100:+.2f}, {yg_cw*100:+.2f})\\text{{ cm}}$ | $({(xg_cw - xg_cw_pred)*100:+.2f}, {(yg_cw - yg_cw_pred)*100:+.2f})\\text{{ cm}}$ |\n")
        f.write(f"| **CCW** | $({xg_ccw_pred*100:+.2f}, {yg_ccw_pred*100:+.2f})\\text{{ cm}}$ | $({xg_ccw*100:+.2f}, {yg_ccw*100:+.2f})\\text{{ cm}}$ | $({(xg_ccw - xg_ccw_pred)*100:+.2f}, {(yg_ccw - yg_ccw_pred)*100:+.2f})\\text{{ cm}}$ |\n\n")

        f.write("### Error Plot and Analysis\n")
        f.write("![UMBmark Return Error Plot](umbmark_plot.png)\n\n")
        f.write(f"- **Simulation Determinism:** CW runs cluster within {spread_cw_mm:.1f} mm and CCW runs within {spread_ccw_mm:.1f} mm, as shown in the insets.\n")
        f.write(f"- **Prediction Accuracy:** The measured return error vectors match the first-order small-angle model to within {max_disc_cm:.2f} cm.\n\n")

        f.write("## [Task 5d] Table 3: Systematic Error Identification (Using Both $\\varepsilon_x$ and $\\varepsilon_y$)\n\n")
        f.write("$$\\alpha + \\beta = \\frac{\\varepsilon_{x,\\text{ccw}} - \\varepsilon_{y,\\text{ccw}}}{4L} = " + f"{sum_ab:+.6f}\\text{{ rad}}$$\n\n")
        f.write("$$\\alpha - \\beta = \\frac{\\varepsilon_{x,\\text{cw}} + \\varepsilon_{y,\\text{cw}}}{4L} = " + f"{diff_ab:+.6f}\\text{{ rad}}$$\n\n")
        f.write("| Systematic Error | Theoretical Prediction | Identified from UMBmark | Accuracy / Difference |\n")
        f.write("| :--- | :---: | :---: | :---: |\n")
        f.write(f"| **Turn Error ($\\alpha$)** | ${alpha_pred:+.6f}\\text{{ rad}}$ (${math.degrees(alpha_pred):+.4f}^\\circ$) | ${alpha_meas:+.6f}\\text{{ rad}}$ (${math.degrees(alpha_meas):+.4f}^\\circ$) | ${math.degrees(alpha_meas - alpha_pred):+.4f}^\\circ$ offset |\n")
        f.write(f"| **Curvature Error ($\\beta$)** | ${beta_pred:+.6f}\\text{{ rad}}$ (${math.degrees(beta_pred):+.4f}^\\circ$) | ${beta_meas:+.6f}\\text{{ rad}}$ (${math.degrees(beta_meas):+.4f}^\\circ$) | {beta_err_pct:.2f}% error |\n\n")

        f.write("## [Task 5d] Table 4: Calibrated Odometry Parameters (Holding Mean Radius at $r_n = 0.053$ m)\n\n")
        f.write("| Parameter | Nameplate Value | Calibrated Value (5d) | Physical Ground Truth | Status / Accuracy |\n")
        f.write("| :--- | :---: | :---: | :---: | :---: |\n")
        f.write(f"| **Track width ($b$)** | ${bn:.6f}\\text{{ m}}$ | **${b_prime:.6f}\\text{{ m}}$** ($b'$) | ${b_act:.6f}\\text{{ m}}$ | $b' = b \\frac{{r_n}}{{\\bar{{r}}}}$ (scale test in 5e) |\n")
        f.write(f"| **Wheel ratio ($E_d = r_R / r_L$)** | $1.000000$ | **${Ed:.6f}$** | ${Ed_true:.6f}$ | {Ed_err_pct:.3f}% error |\n")
        f.write(f"| **Left wheel radius ($r_L$)** | ${rn:.6f}\\text{{ m}}$ | **${r_left_umb:.6f}\\text{{ m}}$** | ${r_L_act:.6f}\\text{{ m}}$ | Unscaled (scaled in 5e) |\n")
        f.write(f"| **Right wheel radius ($r_R$)** | ${rn:.6f}\\text{{ m}}$ | **${r_right_umb:.6f}\\text{{ m}}$** | ${r_R_act:.6f}\\text{{ m}}$ | Unscaled (scaled in 5e) |\n")
        f.write(f"| **Radius difference ($r_L - r_R$)** | $0.000\\text{{ mm}}$ | **${(r_left_umb - r_right_umb)*1e3:.3f}\\text{{ mm}}$** | ${(r_L_act - r_R_act)*1e3:.3f}\\text{{ mm}}$ | {dr_err_mm:.3f} mm error |\n")
    print(f"Saved markdown report to {report_path}")


if __name__ == '__main__':
    main()
