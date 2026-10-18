#!/usr/bin/env python3
"""make_paths.py - (instructor) regenerate the reference paths in paths/.

    python3 tools/make_paths.py            # writes paths/circle.csv, paths/square.csv

Both paths start at the spawn pose (origin, facing +x) and run
counter-clockwise:

    circle.csv   radius 1.5 m, centred at (0, 1.5)
    square.csv   side 2.0 m, corners (0,0), (2,0), (2,2), (0,2)

Points are 1 cm apart along the path; square corners are included exactly.
Both paths are closed: the last point repeats the first. Columns are
x, y (m) and yaw (rad, in (-pi, pi]), the path tangent; at a square corner
yaw is the direction of the leg that starts there.

This needs no ROS. path_publisher.py reads the files it writes.
"""
import csv
import math
import os

SPACING = 0.01
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'paths')


def circle(radius, spacing=SPACING):
    """Circle through the origin, tangent to +x, turning left."""
    n = max(8, math.ceil(2.0 * math.pi * radius / spacing))
    pts = []
    for k in range(n + 1):                      # k = n closes the loop
        phi = 2.0 * math.pi * k / n             # angle turned so far
        pts.append((radius * math.sin(phi), radius * (1.0 - math.cos(phi)), phi))
    return pts


def square(side, spacing=SPACING):
    """Square from the origin along +x, turning left at each corner."""
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    n = max(1, math.ceil(side / spacing))
    pts = []
    for leg, ((x0, y0), (x1, y1)) in enumerate(zip(corners[:-1], corners[1:])):
        yaw = leg * math.pi / 2.0
        for k in range(n + 1 if leg == 3 else n):   # the last leg also adds the end point
            f = k / n
            pts.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw))
    return pts


def write(name, pts):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'yaw'])
        for x, y, yaw in pts:
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            # round, then add 0.0 so round-off never prints as "-0.00000"
            w.writerow([f'{round(x, 5) + 0.0:.5f}', f'{round(y, 5) + 0.0:.5f}',
                        f'{round(yaw, 6) + 0.0:.6f}'])
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))
    print(f'{os.path.normpath(path)}: {len(pts)} points, {length:.4f} m')


if __name__ == '__main__':
    write('circle.csv', circle(1.5))
    write('square.csv', square(2.0))
