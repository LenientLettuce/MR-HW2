#!/usr/bin/env python3
"""Plot a commanded square and both recorded paths from one run's CSV."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


HW2_ROOT = Path(__file__).resolve().parent.parent
TOPICS = ('/odom', '/ground_truth/odom')


def load_paths(csv_path):
    """Read all poses, keeping the two ROS topics separate."""
    paths = {topic: [] for topic in TOPICS}
    with csv_path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        required = {'topic', 'x_m', 'y_m', 'yaw_rad', 'frame_id'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'{csv_path}: expected columns {sorted(required)}')
        frames = {topic: set() for topic in TOPICS}
        for row in reader:
            topic = row['topic']
            if topic not in paths:
                continue
            pose = tuple(float(row[key]) for key in ('x_m', 'y_m', 'yaw_rad'))
            if not all(math.isfinite(value) for value in pose):
                raise ValueError(f'{csv_path}: non-finite pose at CSV line {reader.line_num}')
            paths[topic].append(pose)
            frames[topic].add(row['frame_id'])
    for topic in TOPICS:
        if not paths[topic]:
            raise ValueError(f'{csv_path}: no poses for {topic}')
        if len(frames[topic]) != 1:
            raise ValueError(f'{csv_path}: {topic} changes reference frame')
    return paths


def in_start_frame(poses):
    """Express a path relative to its initial position and forward heading."""
    x0, y0, yaw0 = poses[0]
    c, s = math.cos(yaw0), math.sin(yaw0)
    return [(c * (x - x0) + s * (y - y0),
             -s * (x - x0) + c * (y - y0)) for x, y, _ in poses]


def commanded_square(side, direction):
    """Ideal path; in-place rotations add no position change at the corners."""
    sign = 1.0 if direction == 'ccw' else -1.0
    return [(0.0, 0.0), (side, 0.0), (side, sign * side),
            (0.0, sign * side), (0.0, 0.0)]


def plot_run(csv_path, side, direction, title, output_stem):
    paths = load_paths(csv_path)
    fig, ax = plt.subplots(figsize=(7.2, 7.2), layout='constrained')
    ideal = commanded_square(side, direction)
    ax.plot(*zip(*ideal), color='#343434', linestyle='--', linewidth=1.8,
            label='Commanded square', zorder=1)
    for topic, color, label in [
        ('/odom', '#0072B2', '/odom'),
        ('/ground_truth/odom', '#D55E00', 'True path (/ground_truth/odom)'),
    ]:
        xy = in_start_frame(paths[topic])
        ax.plot(*zip(*xy), color=color, linewidth=1.7, label=label)
        ax.scatter(*xy[-1], color=color, marker='X', s=65, zorder=5)
    ax.scatter(0.0, 0.0, color='#222222', marker='o', s=32, zorder=6,
               label='Start (X markers show final positions)')
    ax.set_title(f'{title}\nSide length {side:g} m, {direction.upper()}', pad=14)
    ax.set_xlabel('x from start (m)')
    ax.set_ylabel('y from start (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, color='#d9d9d9', linewidth=0.6, alpha=0.8)
    ax.set_axisbelow(True)
    ax.margins(0.12)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), frameon=False,
              fontsize=9)
    output_stem.parent.mkdir(parents=True, exist_ok=True)
    for suffix in ('.png', '.pdf'):
        destination = output_stem.with_suffix(suffix)
        fig.savefig(destination, dpi=220)
        print(destination)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path,
                        default=HW2_ROOT / 'Square Loop Runs' / 'square_loop_run4.csv')
    parser.add_argument('--side', type=float, default=2.0,
                        help='Commanded L in metres; not stored in the pose CSV')
    parser.add_argument('--direction', choices=('cw', 'ccw'), default='ccw')
    parser.add_argument('--title', default='Square loop: run 1',
                        help='Use the robot/pair identifier if known')
    parser.add_argument('--output', type=Path, default=None,
                        help='Output file stem, without .png/.pdf')
    args = parser.parse_args()
    if not math.isfinite(args.side) or args.side <= 0:
        parser.error('--side must be finite and greater than zero')
    output = args.output or Path(__file__).resolve().parent / args.csv.stem
    plot_run(args.csv, args.side, args.direction, args.title, output)


if __name__ == '__main__':
    main()
