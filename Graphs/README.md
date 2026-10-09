# Square loop graphs

`plot_square.py` plots one run's commanded square, `/odom` path, and true
`/ground_truth/odom` path on the same axes with equal scaling. It reads all
rows with Python's CSV reader and exports PNG and PDF files in this folder.
It does not require ROS to be running. Python needs `matplotlib` installed.

From the HW2 root, reproduce the run 1 graph:

```bash
python3 Graphs/plot_square.py
```

Plot a different run or robot:

```bash
python3 Graphs/plot_square.py \
  --csv 'Square Loop Runs/square_loop_run2.csv' \
  --side 2.0 --direction ccw --title 'Square loop: run 2'
```

Use `--title` to identify the actual robot, for example REF or your pair ID.
The pose CSV does not identify the robot or record the commanded side length.
Defaults of 2 m and CCW match the node defaults and the previously discussed
run commands; set these options to the actual parameters if you changed them.
An ideal commanded square is reconstructed from those parameters, rather
than measured from velocity messages (which are not recorded in the CSV).

Each recorded path is translated and rotated into its own starting frame:
the start is (0, 0), and the initial forward direction is +x. This handles
the difference between the odometry and world frames for nonzero spawn
poses. It does not rescale, smooth, or adjust the final trajectory. These
three existing CSVs start essentially at the origin with zero heading, so
this transformation has negligible numerical effect on them.

X markers show the final poses. Post-stop duplicate poses are retained and
do not change the line. In-place turns do not add segments to the ideal
square. To satisfy "one run of each robot," use a CSV from each robot;
different runs alone do not establish that different robots were used.
