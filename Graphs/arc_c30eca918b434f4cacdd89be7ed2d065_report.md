# P08 arc: Euler vs Exact

### Command-end endpoint (last matched sample at/before stop)

Elapsed simulation time: 29.999 s.

| Source | x (m) | y (m) | yaw (deg) | Position error vs GT (mm) | Heading error vs GT (deg) |
|---|---:|---:|---:|---:|---:|
| Euler | -0.382250882 | 0.074992050 | -22.086484 | 3.241431 | 0.337824 |
| Exact | -0.382324356 | 0.074615818 | -22.086484 | 3.424318 | 0.337824 |
| Ground truth (interpolated) | -0.384852898 | 0.076925024 | -22.424308 | 0.000000 | 0.000000 |

Euler minus Exact: dx=0.000073474 m, dy=0.000376232 m, separation=0.383339 mm, heading difference=0.000000000 deg.

### Final recorded endpoint (including settling)

Elapsed simulation time: 30.999 s.

| Source | x (m) | y (m) | yaw (deg) | Position error vs GT (mm) | Heading error vs GT (deg) |
|---|---:|---:|---:|---:|---:|
| Euler | -0.381695152 | 0.074766544 | -22.052689 | 1.548104 | 0.261225 |
| Exact | -0.381768559 | 0.074390475 | -22.052689 | 1.930424 | 0.261225 |
| Ground truth (interpolated) | -0.381512045 | 0.076303781 | -22.313913 | 0.000000 | 0.000000 |

Euler minus Exact: dx=0.000073408 m, dy=0.000376068 m, separation=0.383166 mm, heading difference=0.000000000 deg.

### Implications

Euler translates using the heading at the start of each encoder interval. Exact follows the circular arc implied by that interval’s wheel travel. Both accumulate the same heading changes, so their heading difference should be zero. Euler’s position error depends on the integration timestep, turning rate, and path length. These estimates integrate at a target of 100 Hz using cumulative wheel travel between integration steps; publication frequency is separate. A higher integration rate reduces Euler’s approximation error. A nearly complete circle can also partly cancel endpoint error, so a small final difference does not imply small error everywhere. Exact removes this discretization approximation under the constant-curvature interval model; it does not remove wheel slip or incorrect wheel dimensions.

Ground truth is expressed relative to the recorded start pose and interpolated at the matched comparison timestamp. Missing timestamp coverage is reported as n.a.
