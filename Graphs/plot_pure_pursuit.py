#!/usr/bin/env python3
"""Overlay pure pursuit runs sharing a reference path, with cross-track errors."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SOURCES = (
    ('/odom', '#0072B2', '/odom'),
    ('/ground_truth/odom', '#D55E00', 'Ground truth'),
)


def load_run(csv_path):
    metadata_path = csv_path.with_suffix('.json')
    metadata = json.loads(metadata_path.read_text(encoding='utf-8')) if metadata_path.exists() else {}
    paths = {topic: [] for topic, _, _ in SOURCES}
    with csv_path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        required = {'topic', 'run_time_s', 'x_m', 'y_m', 'cross_track_error_m'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'{csv_path}: expected columns {sorted(required)}')
        for row in reader:
            if row['topic'] not in paths:
                continue
            sample = tuple(float(row[key]) for key in
                           ('run_time_s', 'x_m', 'y_m', 'cross_track_error_m'))
            if not all(math.isfinite(value) for value in sample):
                raise ValueError(f'{csv_path}: non-finite sample at line {reader.line_num}')
            paths[row['topic']].append(sample)
    for topic in paths:
        if not paths[topic]:
            raise ValueError(f'{csv_path}: no samples for {topic}')
        paths[topic].sort(key=lambda sample: sample[0])
    # The logger already transformed GT into odom; do not align each path
    # to its own first sample, which would hide position errors.
    reference = metadata.get('reference_waypoints')
    if not reference:
        path_type = metadata.get('path', csv_path.stem.split('_run')[0])
        if path_type not in ('circle', 'square'):
            raise ValueError('Cannot determine reference path; retain the companion JSON')
        with (ROOT / 'src_mustafa' / 'paths' / f'{path_type}.csv').open(newline='') as stream:
            reference = [(float(row['x']), float(row['y'])) for row in csv.DictReader(stream)]
    reference = [(float(point[0]), float(point[1])) for point in reference]
    if not reference or not all(math.isfinite(v) for point in reference for v in point):
        raise ValueError('Invalid reference waypoints')
    if reference[-1] != reference[0]:
        reference.append(reference[0])
    return metadata, paths, reference


def plot_runs(csv_paths, output, title=None, source=None):
    runs = [(csv_path, *load_run(csv_path)) for csv_path in csv_paths]
    reference = runs[0][3]
    frame = runs[0][1].get('comparison_frame', 'odom')
    for csv_path, metadata, _, candidate in runs[1:]:
        if (len(candidate) != len(reference)
                or any(not math.isclose(a, b, rel_tol=0, abs_tol=1e-9)
                       for left, right in zip(reference, candidate)
                       for a, b in zip(left, right))):
            raise ValueError(f'{csv_path}: reference path differs from {csv_paths[0]}')
        if metadata.get('comparison_frame', 'odom') != frame:
            raise ValueError(f'{csv_path}: comparison frame differs from {csv_paths[0]}')
    # A multi-run comparison shows each measured true path by default.
    # Preserve the original odom + truth view for a single input.
    source = source or ('ground-truth' if len(runs) > 1 else 'both')
    selected = [entry for entry in SOURCES
                if source == 'both'
                or entry[0] == ('/odom' if source == 'odom' else '/ground_truth/odom')]
    fig, (trajectory, error) = plt.subplots(1, 2, figsize=(13, 6), layout='constrained')
    trajectory.plot(*zip(*reference), '--', color='#343434', linewidth=1.8,
                    label='Reference path (shared)')
    colors = plt.get_cmap('tab10')
    for index, (csv_path, metadata, paths, _) in enumerate(runs):
        details = []
        if 'pair' in metadata:
            details.append(f"pair {metadata['pair']}")
        if 'Ld_m' in metadata:
            details.append(f"Ld={metadata['Ld_m']:g} m")
        if 'v_m_s' in metadata:
            details.append(f"v={metadata['v_m_s']:g} m/s")
        run_label = csv_path.stem + (' — ' + ', '.join(details) if details else '')
        for topic, default_color, topic_label in selected:
            color = colors(index % 10) if len(runs) > 1 else default_color
            style = ':' if source == 'both' and topic == '/odom' else '-'
            label = f'{run_label}: {topic_label}' if len(runs) > 1 else topic_label
            times, xs, ys, errors = zip(*paths[topic])
            trajectory.plot(xs, ys, linestyle=style, color=color, linewidth=1.5, label=label)
            trajectory.scatter(xs[-1], ys[-1], color=color, marker='X', s=55, zorder=5)
            error.plot(times, errors, linestyle=style, color=color, linewidth=1.4, label=label)
    trajectory.set(title='Paths (X marks final position)', xlabel=f'x in {frame} frame (m)',
                   ylabel=f'y in {frame} frame (m)')
    trajectory.set_aspect('equal', adjustable='box')
    error.set(title='Cross-track error', xlabel='Elapsed simulation time (s)',
              ylabel='Unsigned distance to reference path (m)')
    error.set_ylim(bottom=0)
    for ax in (trajectory, error):
        ax.grid(True, alpha=0.3)
        ax.set_axisbelow(True)
        ax.legend(fontsize=8)
    default_title = run_label if len(runs) == 1 else f'Pure pursuit: {len(runs)} runs'
    fig.suptitle(title or default_title)
    output.parent.mkdir(parents=True, exist_ok=True)
    for suffix in ('.png', '.pdf'):
        destination = output.with_suffix(suffix)
        fig.savefig(destination, dpi=220)
        print(destination)
    plt.close(fig)


def plot_run(csv_path, output, title=None):
    """Retain the single-run entry point for callers importing this script."""
    plot_runs([csv_path], output, title)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, nargs='+', required=True,
                        help='One or more run CSVs sharing the same reference path')
    parser.add_argument('--source', choices=('ground-truth', 'odom', 'both'),
                        help='Default: ground-truth for multiple runs, both for one run')
    parser.add_argument('--output', type=Path, help='Output stem; defaults to Graphs/<CSV stem>')
    parser.add_argument('--title', help='Optional figure title')
    args = parser.parse_args()
    stem = args.csv[0].stem + ('_comparison' if len(args.csv) > 1 else '')
    output = args.output or Path(__file__).resolve().parent / stem
    try:
        plot_runs(args.csv, output, args.title, args.source)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(1, f'Error: {exc}\n')


if __name__ == '__main__':
    main()
