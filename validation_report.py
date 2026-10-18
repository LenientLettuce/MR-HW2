#!/usr/bin/env python3
"""validation_report.py - Task 5(f): compare calibrated end errors with the uncalibrated 5(c) results.

The uncalibrated baseline is computed from the 5(c) CSV (centre of gravity per direction),
never typed in. If a file is missing the script says so and does not invent numbers.

Usage:
    python3 validation_report.py --calibrated umbmark_calibrated.csv --baseline umbmark.csv
"""
import argparse
import csv
import math
import sys
from pathlib import Path


def load(path):
    rows = {"cw": [], "ccw": []}
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            d = r["direction"].strip().lower()
            if d in rows:
                rows[d].append((float(r["eps_x"]) * 100, float(r["eps_y"]) * 100,
                                math.degrees(float(r["eps_theta"]))))
    return rows


def mean(vals):
    return sum(vals) / len(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrated", default="umbmark_calibrated.csv")
    ap.add_argument("--baseline", default="umbmark.csv")
    a = ap.parse_args()

    if not Path(a.calibrated).exists():
        sys.exit(f"ERROR: {a.calibrated} not found; the validation runs did not produce data.")
    cal = load(a.calibrated)
    base = load(a.baseline) if Path(a.baseline).exists() else None
    if base is None:
        print(f"WARNING: {a.baseline} not found, so no uncalibrated baseline is shown.\n")

    line = "+" + "-" * 11 + "+" + "-" * 24 + "+" + "-" * 38 + "+" + "-" * 12 + "+"
    print(line)
    print(f"| {'Direction':<9} | {'Uncalibrated c.g. (cm)':<22} | {'Calibrated end error':<36} | {'|eps| drop':<10} |")
    print(line)
    for d in ("cw", "ccw"):
        if not cal[d]:
            print(f"| {d.upper():<9} | {'(no calibrated run recorded)':<73} |")
            continue
        if base and base[d]:
            bx = mean([r[0] for r in base[d]])
            by = mean([r[1] for r in base[d]])
            bdist = math.hypot(bx, by)
            btxt = f"({bx:+6.2f}, {by:+6.2f}) |e|={bdist:5.2f}"
        else:
            bdist, btxt = None, "n/a"
        for i, (ex, ey, eth) in enumerate(cal[d], 1):
            dist = math.hypot(ex, ey)
            drop = f"{(1 - dist / bdist) * 100:5.1f} %" if bdist else "n/a"
            ctxt = f"({ex:+6.2f}, {ey:+6.2f}) cm |e|={dist:5.2f} cm th={eth:+6.2f} deg"
            print(f"| {d.upper() + ' #' + str(i):<9} | {btxt:<22} | {ctxt:<36} | {drop:<10} |")
    print(line)
    n_cw, n_ccw = len(cal["cw"]), len(cal["ccw"])
    print(f"Calibrated runs recorded: {n_cw} CW, {n_ccw} CCW")
    if base:
        print(f"Baseline runs used: {len(base['cw'])} CW, {len(base['ccw'])} CCW (mean per direction)")


if __name__ == "__main__":
    main()