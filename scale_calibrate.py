#!/usr/bin/env python3
"""scale_calibrate.py - Apply the scale-test result to the UMBmark parameters (Task 5e).

Reads scale_test.csv (columns d_odom, d_true, c; one row per run), averages c,
scales the after-UMBmark parameters, prints the four-stage table, and writes the
calibrated odometry parameter file for Task 5(f).

There is deliberately NO fallback: if the CSV is missing or empty the script stops,
so a table can never be built from anything but measured data.

Usage:
    python3 scale_calibrate.py --csv scale_test.csv --b-prime 0.313626 --Ed 0.994480
Pass --b-prime and --Ed from your latest UMBmark analysis output.

[Tool note for the report: edited with AI assistance.]
"""
import argparse
import csv
import statistics
import sys
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser(description="Calibrate odometry using the scale test.")
    p.add_argument("--csv", default="scale_test.csv")
    p.add_argument("--rn", type=float, default=0.053000, help="nameplate radius [m]")
    p.add_argument("--bn", type=float, default=0.313000, help="nameplate track width [m]")
    p.add_argument("--b-prime", type=float, default=None, help="UMBmark track width b' [m]")
    p.add_argument("--Ed", type=float, default=None, help="UMBmark wheel ratio Ed = rR/rL")
    p.add_argument("--r-left-act", type=float, default=0.054146, help="actual rL [m]")
    p.add_argument("--r-right-act", type=float, default=0.053854, help="actual rR [m]")
    p.add_argument("--b-act", type=float, default=0.319000, help="actual b [m]")
    p.add_argument("--method", default="rk2", help="integration method string your node expects")
    p.add_argument("--node-name", default="odometry_node", help="node name used as the YAML key")
    p.add_argument("--yaml-path", default="src_affan/config/odometry_calibrated.yaml")
    p.add_argument("--no-yaml", action="store_true", help="do not write the YAML file")
    return p.parse_args()


def load_scale_test(path):
    if not path.exists():
        sys.exit(f"ERROR: {path} not found. Run the scale test first; no fallback is used.")
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"ERROR: {path} has no data rows. Run the scale test first.")
    d_odom = [float(r["d_odom"]) for r in rows]
    d_true = [float(r["d_true"]) for r in rows]
    cs = [float(r["c"]) for r in rows]
    return d_odom, d_true, cs


def main():
    a = parse_args()
    d_odom, d_true, cs = load_scale_test(Path(a.csv))
    n = len(cs)
    c = statistics.mean(cs)
    c_sd = statistics.stdev(cs) if n > 1 else float("nan")

    b_prime, Ed = a.b_prime, a.Ed
    if b_prime is None or Ed is None:
        b_prime = 0.313626 if b_prime is None else b_prime
        Ed = 0.994480 if Ed is None else Ed
        print("WARNING: using default b' and/or Ed. Confirm they match your latest UMBmark "
              "output, or pass --b-prime and --Ed.\n")

    rn = a.rn
    rL_umb = 2.0 * rn / (1.0 + Ed)          # after UMBmark, mean radius kept at rn
    rR_umb = Ed * rL_umb

    rbar_f = c * rn                          # after the scale test
    rL_f = c * rL_umb
    rR_f = c * rR_umb
    b_f = c * b_prime
    Ed_f = rR_f / rL_f

    rbar_act = (a.r_left_act + a.r_right_act) / 2.0
    Ed_act = a.r_right_act / a.r_left_act

    sep = "=" * 100
    print(sep)
    print("TASK 5(e): FOUR-STAGE PARAMETER TABLE")
    print(sep)
    print(f"Scale test: {n} run(s), mean d_odom = {statistics.mean(d_odom):.5f} m, "
          f"mean d_true = {statistics.mean(d_true):.5f} m")
    sd_txt = f"{c_sd:.6f}" if n > 1 else "n/a (single run)"
    print(f"c = d_true / d_odom: mean {c:.6f}, sd {sd_txt}   (r_bar_actual / rn = {rbar_act / rn:.6f})")
    print("-" * 100)
    hdr = f"{'Parameter':<22}{'Nameplate':>16}{'After UMBmark':>18}{'After scale test':>20}{'Actual':>16}"
    print(hdr)
    print("-" * len(hdr))

    def line(name, v0, v1, v2, v3, unit="m", fmt="{:.6f}"):
        print(f"{name:<22}{fmt.format(v0) + ' ' + unit:>16}{fmt.format(v1) + ' ' + unit:>18}"
              f"{fmt.format(v2) + ' ' + unit:>20}{fmt.format(v3) + ' ' + unit:>16}")

    line("Track width b", a.bn, b_prime, b_f, a.b_act)
    line("Left radius rL", rn, rL_umb, rL_f, a.r_left_act)
    line("Right radius rR", rn, rR_umb, rR_f, a.r_right_act)
    line("Mean radius r_bar", rn, rn, rbar_f, rbar_act)
    line("Ratio Ed = rR/rL", 1.0, Ed, Ed_f, Ed_act, unit="", fmt="{:.6f}")
    print("-" * len(hdr))
    print("Residual error after the scale test (calibrated - actual):")
    print(f"  b     : {(b_f - a.b_act) * 1e3:+.3f} mm")
    print(f"  rL    : {(rL_f - a.r_left_act) * 1e3:+.3f} mm")
    print(f"  rR    : {(rR_f - a.r_right_act) * 1e3:+.3f} mm")
    print(f"  r_bar : {(rbar_f - rbar_act) * 1e3:+.3f} mm")
    print(f"  Ed    : {Ed_f - Ed_act:+.6f}")
    print(sep + "\n")

    if not a.no_yaml:
        path = Path(a.yaml_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write("# Calibrated odometry parameters after UMBmark + scale test (Task 5e)\n")
            f.write(f"{a.node_name}:\n  ros__parameters:\n    use_sim_time: true\n")
            f.write(f"    r_left: {rL_f:.6f}\n    r_right: {rR_f:.6f}\n    b: {b_f:.6f}\n")
            f.write(f"    method: {a.method}\n")
        print(f"Wrote calibrated parameters to {path}")


if __name__ == "__main__":
    main()