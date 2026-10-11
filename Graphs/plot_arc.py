#!/usr/bin/env python3
"""Plot an arc CSV and report matched Euler/Exact endpoint differences."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
TOPICS = ('/odom_euler', '/odom_exact', '/ground_truth/odom')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def load_run(path):
    metadata_path = path.with_suffix('.json')
    if not metadata_path.is_file():
        raise ValueError(f'Missing run metadata: {metadata_path}')
    metadata = json.loads(metadata_path.read_text())
    paths = {topic: {} for topic in TOPICS}
    with path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        required = {'topic', 'stamp_sec', 'stamp_nanosec', 'x_m', 'y_m', 'yaw_rad', 'frame_id'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'Expected CSV columns: {sorted(required)}')
        frames = {topic: set() for topic in TOPICS}
        for row in reader:
            topic = row['topic']
            if topic not in paths:
                continue
            pose = tuple(float(row[k]) for k in ('x_m', 'y_m', 'yaw_rad'))
            if not all(math.isfinite(v) for v in pose):
                raise ValueError(f'Nonfinite pose at line {reader.line_num}')
            timestamp = int(row['stamp_sec']) * 1_000_000_000 + int(row['stamp_nanosec'])
            paths[topic][timestamp] = pose
            frames[topic].add(row['frame_id'])
    for topic in TOPICS:
        if not paths[topic] or len(frames[topic]) != 1:
            raise ValueError(f'Missing poses or changing frame for {topic}')
    if frames['/odom_euler'] != {'arc_start'} or frames['/odom_exact'] != {'arc_start'}:
        raise ValueError('Euler/Exact poses must use the shared arc_start frame')
    origin = metadata.get('ground_truth_start')
    if origin is None or metadata.get('start_ns') is None:
        raise ValueError('Run has no recorded starting pose/time')
    x0, y0, yaw0 = origin
    c, s = math.cos(yaw0), math.sin(yaw0)
    paths['/ground_truth/odom'] = {
        t: (c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0), wrap(yaw - yaw0))
        for t, (x, y, yaw) in paths['/ground_truth/odom'].items()
    }
    matched = sorted(paths['/odom_euler'].keys() & paths['/odom_exact'].keys())
    if len(matched) < 2:
        raise ValueError('Need at least two matched Euler/Exact timestamps')
    return metadata, paths, matched


def interpolate(poses, times, timestamp):
    """Interpolate GT at the comparison timestamp; do not extrapolate."""
    i = bisect.bisect_left(times, timestamp)
    if i < len(times) and times[i] == timestamp:
        return poses[timestamp]
    if i == 0 or i == len(times):
        return None
    t0, t1 = times[i - 1], times[i]
    a, b = poses[t0], poses[t1]
    fraction = (timestamp - t0) / (t1 - t0)
    return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]),
            wrap(a[2] + fraction * wrap(b[2] - a[2])))


def endpoint_report(metadata, paths, matched):
    sections = []
    rate = metadata.get('integration_rate')
    integration_note = (
        f'These estimates integrate at a target of {rate:g} Hz using cumulative wheel travel '
        'between integration steps; publication frequency is separate. '
        if rate is not None else
        'These estimates integrate every joint-state sample, not just the 50 Hz publications. '
    )
    gt_times = sorted(paths['/ground_truth/odom'])
    endpoints = [('Final recorded endpoint (including settling)', matched[-1])]
    if metadata.get('stop_ns') is not None:
        driving = [t for t in matched if t <= metadata['stop_ns']]
        if driving:
            endpoints.insert(0, ('Command-end endpoint (last matched sample at/before stop)', driving[-1]))
    for title, timestamp in endpoints:
        euler, exact = paths['/odom_euler'][timestamp], paths['/odom_exact'][timestamp]
        gt = interpolate(paths['/ground_truth/odom'], gt_times, timestamp)
        lines = [f'### {title}', '',
                 f'Elapsed simulation time: {(timestamp - metadata["start_ns"]) / 1e9:.3f} s.', '',
                 '| Source | x (m) | y (m) | yaw (deg) | Position error vs GT (mm) | Heading error vs GT (deg) |',
                 '|---|---:|---:|---:|---:|---:|']
        for label, pose in [('Euler', euler), ('Exact', exact), ('Ground truth (interpolated)', gt)]:
            if pose is None:
                lines.append(f'| {label} | n.a. | n.a. | n.a. | n.a. | n.a. |')
                continue
            error = f'{1000 * math.hypot(pose[0]-gt[0], pose[1]-gt[1]):.6f}' if gt else 'n.a.'
            yaw_error = f'{abs(math.degrees(wrap(pose[2]-gt[2]))):.6f}' if gt else 'n.a.'
            lines.append(f'| {label} | {pose[0]:.9f} | {pose[1]:.9f} | '
                         f'{math.degrees(pose[2]):.6f} | {error} | {yaw_error} |')
        distance = math.hypot(euler[0]-exact[0], euler[1]-exact[1])
        lines += ['', f'Euler minus Exact: dx={euler[0]-exact[0]:.9f} m, '
                  f'dy={euler[1]-exact[1]:.9f} m, separation={distance * 1000:.6f} mm, '
                  f'heading difference={math.degrees(wrap(euler[2]-exact[2])):.9f} deg.']
        sections.append('\n'.join(lines))
    return '\n\n'.join(sections)


def plot_run(csv_path, output, title):
    metadata, paths, matched = load_run(csv_path)
    fig, (trajectory, endpoint, detail) = plt.subplots(
        1, 3, figsize=(16, 5.6), layout='constrained')
    v, omega, duration = (metadata[k] for k in ('v', 'omega', 'duration'))
    ts = [duration * i / 600 for i in range(601)]
    ideal = [(v / omega * math.sin(omega*t), v / omega * (1-math.cos(omega*t))) for t in ts]
    trajectory.plot(*zip(*ideal), '--', color='#555555', label='Commanded arc')
    sources = [('/odom_euler', '#0072B2', 'Euler'),
               ('/odom_exact', '#009E73', 'Exact'),
               ('/ground_truth/odom', '#D55E00', 'Ground truth')]
    for topic, color, label in sources:
        xy = [paths[topic][t][:2] for t in sorted(paths[topic])]
        trajectory.plot(*zip(*xy), color=color, linewidth=1.6, label=label)
    trajectory.set(xlabel='x from arc start (m)', ylabel='y from arc start (m)',
                   title='Arc overview')
    trajectory.margins(0.025)
    trajectory.legend(loc='center', fontsize=9)

    # Use one timestamp for all zoom markers, with GT interpolation rather
    # than mismatched final samples. Retain the latest paired endpoint if
    # ground-truth coverage is unavailable.
    gt_times = sorted(paths['/ground_truth/odom'])
    covered = [t for t in matched if gt_times[0] <= t <= gt_times[-1]]
    timestamp = covered[-1] if covered else matched[-1]
    gt = interpolate(paths['/ground_truth/odom'], gt_times, timestamp)
    euler = paths['/odom_euler'][timestamp]
    exact = paths['/odom_exact'][timestamp]
    elapsed = (timestamp - metadata['start_ns']) / 1e9
    poses = {'/odom_euler': euler, '/odom_exact': exact, '/ground_truth/odom': gt}
    for ax in (endpoint, detail):
        ax.axhline(0, color='#cccccc', linewidth=.8, zorder=0)
        ax.axvline(0, color='#cccccc', linewidth=.8, zorder=0)
    endpoint_points = []
    for topic, color, label in sources:
        pose = poses[topic]
        if pose is None:
            continue
        xy = (1000 * (pose[0]-exact[0]), 1000 * (pose[1]-exact[1]))
        endpoint_points.append(xy)
        # Plot the actual nearby path, clipped by the endpoint zoom limits.
        local_path = [paths[topic][t][:2] for t in sorted(paths[topic])
                      if timestamp - 1_000_000_000 <= t <= timestamp]
        if local_path:
            endpoint.plot([1000*(x-exact[0]) for x, y in local_path],
                          [1000*(y-exact[1]) for x, y in local_path],
                          color=color, linewidth=1.2, alpha=.75)
        endpoint.scatter(*xy, color=color, marker='X', s=60, label=label, zorder=4)
        endpoint.annotate(label, xy, xytext=(7, 7 if topic != '/odom_exact' else -16),
                          textcoords='offset points', color=color, fontsize=9)
    endpoint.set(xlabel='x relative to final Exact pose (mm)',
                 ylabel='y relative to final Exact pose (mm)',
                 title=f'Final-pose zoom ({elapsed:.3f} s)')

    dx, dy = euler[0]-exact[0], euler[1]-exact[1]
    separation = math.hypot(dx, dy)
    scale, unit = (1e6, 'µm') if separation < .001 else (1000., 'mm')
    offset = (dx*scale, dy*scale)
    for xy, color, label in [((0., 0.), '#009E73', 'Exact'),
                              (offset, '#0072B2', 'Euler')]:
        detail.scatter(*xy, color=color, marker='X', s=80, zorder=4)
        detail.annotate(label, xy, xytext=(8, 8), textcoords='offset points',
                        color=color, fontsize=10)
    if separation > 0:
        detail.annotate('', xy=offset, xytext=(0., 0.),
                        arrowprops={'arrowstyle': '<->', 'color': '#333333', 'lw': 1.3})
    detail.set(xlabel=f'x relative to final Exact pose ({unit})',
               ylabel=f'y relative to final Exact pose ({unit})',
               title=f'Euler vs Exact: {separation*scale:.3f} {unit}')
    detail.text(.04, .96, f'Δx = {dx*scale:.3f} {unit}\nΔy = {dy*scale:.3f} {unit}',
                transform=detail.transAxes, va='top', fontsize=10,
                bbox={'facecolor': 'white', 'edgecolor': '#dddddd', 'alpha': .9})

    def zoom_limits(ax, points, minimum_span):
        xs, ys = zip(*points)
        span = max(max(xs)-min(xs), max(ys)-min(ys), minimum_span)
        center_x, center_y = (max(xs)+min(xs))/2, (max(ys)+min(ys))/2
        half = span * .8
        ax.set_xlim(center_x-half, center_x+half)
        ax.set_ylim(center_y-half, center_y+half)

    zoom_limits(endpoint, endpoint_points, .1)
    zoom_limits(detail, [(0., 0.), offset], 1. if unit == 'µm' else .001)
    for ax in (trajectory, endpoint, detail):
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=.25)
        ax.tick_params(labelsize=9)
    rate_label = (f', integration={metadata["integration_rate"]:g} Hz'
                  if 'integration_rate' in metadata else '')
    fig.suptitle(f'{title}: v={v:g} m/s, omega={omega:g} rad/s, {duration:g} s{rate_label}')
    output.parent.mkdir(parents=True, exist_ok=True)
    for suffix in ('.png', '.pdf'):
        destination = output.with_suffix(suffix)
        fig.savefig(destination, dpi=220)
        print(destination)
    plt.close(fig)
    report = endpoint_report(metadata, paths, matched)
    report_path = output.with_name(output.name + '_report').with_suffix('.md')
    report_path.write_text(f'# {title}\n\n' + report + '\n', encoding='utf-8')
    print(report)
    print(report_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, default=None, help='Arc CSV; default latest run in Arc Runs')
    parser.add_argument('--output', type=Path, default=None, help='Output stem for PNG/PDF/report')
    parser.add_argument('--title', default='P08 arc: Euler vs Exact')
    args = parser.parse_args()
    if args.csv is None:
        runs = list((ROOT / 'Arc Runs').glob('arc_*.csv'))
        if not runs:
            parser.error('No arc runs found; run arc_run.launch.py first')
        args.csv = max(runs, key=lambda p: p.stat().st_mtime)
    output = args.output or Path(__file__).resolve().parent / args.csv.stem
    try:
        plot_run(args.csv, output, args.title)
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
