#!/usr/bin/env python3
"""Drive four timed straight/turn pairs and log both odometry topics.

The robot remains stopped and logging continues after the square finishes.
See ../OPEN_LOOP_SQUARE_SETUP.md.
"""

import csv
import math
import signal
from pathlib import Path
from threading import Event
from uuid import uuid4

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


def pose_of(msg):
    """Return x, y, yaw from an Odometry message (yaw in radians)."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw


class OpenLoopSquare(Node):
    def __init__(self):
        super().__init__('open_loop_square')
        self.L = self.declare_parameter('L', 2.0).value
        self.v = self.declare_parameter('v', 0.2).value
        self.omega = self.declare_parameter('omega', 0.5).value
        self.direction = self.declare_parameter('direction', 'ccw').value
        for name, value in [('L', self.L), ('v', self.v), ('omega', self.omega)]:
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and greater than zero.')
        if self.direction not in ('cw', 'ccw'):
            raise ValueError("direction must be 'cw' or 'ccw'.")
        self.straight_duration = self.L / self.v
        self.turn_duration = (math.pi / 2.0) / self.omega
        self.odom = self.gt = None
        self.create_csv_log()

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 50)
        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.on_gt, 50)
        
        self.phase = 'start'
        self.leg = 0
        self.phase_start = None
        self.sign = 1.0 if self.direction == 'ccw' else -1.0
        self.timer = self.create_timer(0.02, self.tick)

        self.get_logger().info(
            f'Logging both odometry topics; driving a {self.L} m square '
            f'{self.direction} using simulation-clock durations.'
        )

    def create_csv_log(self):
        """Create a new CSV under the repository's Square Loop Runs folder."""
        repository_root = next(
            (parent for parent in Path(__file__).resolve().parents
             if (parent / 'src_mustafa' / 'package.xml').is_file()
             and (parent / 'src_mustafa' / 'scripts' / 'open_loop_square.py').is_file()),
            None,
        )
        if repository_root is None:
            raise RuntimeError('Cannot locate the HW2 repository root for CSV output.')

        output_folder = repository_root / 'Square Loop Runs'
        output_folder.mkdir(parents=True, exist_ok=True)
        # A unique filename keeps previous runs intact.
        self.csv_path = output_folder / f'square_loop_{uuid4().hex}.csv'
        self.csv_file = self.csv_path.open('x', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.save_csv_row([
            'topic', 'sim_time_s', 'stamp_sec', 'stamp_nanosec',
            'frame_id', 'child_frame_id', 'x_m', 'y_m', 'z_m',
            'qx', 'qy', 'qz', 'qw', 'yaw_rad',
        ])
        self.get_logger().info(f'CSV output: {self.csv_path}')

    def save_csv_row(self, row):
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def log_pose(self, topic, msg):
        """Log one received pose, preserving its source and original timestamp.

        Each topic gets its own rows; messages are not assumed synchronized.
        Positions stay in the frame identified by frame_id.
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp
        sim_time_s = self.get_clock().now().nanoseconds / 1e9
        self.save_csv_row([
            topic, sim_time_s, stamp.sec, stamp.nanosec,
            msg.header.frame_id, msg.child_frame_id,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w, pose_of(msg)[2],
        ])

    def on_odom(self, msg):
        self.odom = pose_of(msg)
        self.log_pose('/odom', msg)

    def on_gt(self, msg):
        self.gt = pose_of(msg)
        self.log_pose('/ground_truth/odom', msg)

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0 or self.odom is None or self.gt is None:
            return

        if self.phase == 'start':
            self.phase_start = now
            self.phase = 'straight'

        elapsed = (now - self.phase_start).nanoseconds / 1e9
        # Transition before selecting the command, so the new phase's
        # velocity is published immediately (including the final stop).
        if self.phase == 'straight' and elapsed >= self.straight_duration:
            self.phase = 'turn'
            self.phase_start = now
        elif self.phase == 'turn' and elapsed >= self.turn_duration:
            self.leg += 1
            self.phase = 'done' if self.leg == 4 else 'straight'
            self.phase_start = now
            if self.phase == 'done':
                self.get_logger().info('Square complete: four sides and turns; stopped.')

        cmd = Twist()
        if self.phase == 'straight':
            cmd.linear.x = self.v
        elif self.phase == 'turn':
            cmd.angular.z = self.sign * self.omega
        # In 'done', the default Twist has zero linear and angular velocity.
        self.cmd.publish(cmd)


def main(args=None):
    # Handle signals explicitly, keeping ROS alive until cleanup sends stop.
    # A bounded spin_once also lets us exit while simulation time is paused.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = Event()
    previous_handlers = {}

    def request_stop(signum, frame):
        stop_requested.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_stop)

    node = None
    try:
        node = OpenLoopSquare()
        while not stop_requested.is_set() and node.context.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.timer.cancel()
                    if node.context.ok():
                        node.cmd.publish(Twist())
                    node.get_logger().info('Stopping node and closing CSV.')
                finally:
                    try:
                        node.csv_file.close()
                    finally:
                        node.destroy_node()
        finally:
            try:
                rclpy.try_shutdown()
            finally:
                for signum, handler in previous_handlers.items():
                    signal.signal(signum, handler)


if __name__ == '__main__':
    main()
