#!/usr/bin/env python3
"""Drive an arc and compare Euler/Exact using identical encoder samples."""

import csv
import json
import math
import signal
from pathlib import Path
from threading import Event
from uuid import uuid4

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from odometry_node import OdometryNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class IntegrationState:
    """Reuse the assignment node's integration equations without another TF."""

    def __init__(self, method):
        self.method = method
        self.x = self.y = self.theta = 0.0

    def update(self, ds, dtheta):
        if dtheta == 0.0:
            # Straight/stationary limit, needed before and after the arc.
            self.x += ds * math.cos(self.theta)
            self.y += ds * math.sin(self.theta)
        else:
            OdometryNode.integrate_pose(self, ds, dtheta)


class ArcRun(Node):
    def __init__(self):
        super().__init__('arc_run')
        defaults = {'v': 0.2, 'omega': 0.2, 'duration': 30.0,
                    'settle_time': 1.0, 'publish_rate': 50.0, 'integration_rate': 100.0,
                    'r_left': 0.054146, 'r_right': 0.053854, 'b': 0.319}
        self.settings = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for key, value in self.settings.items():
            if not math.isfinite(value):
                raise ValueError(f'{key} must be finite')
            if key not in ('v', 'omega') and value <= 0:
                raise ValueError(f'{key} must be positive')
        if self.settings['v'] == 0 or self.settings['omega'] == 0:
            raise ValueError('v and omega must be nonzero for an arc')
        self.period_ns = max(1, round(1e9 / self.settings['publish_rate']))
        self.integration_period_ns = max(1, round(1e9 / self.settings['integration_rate']))
        self.states = {method: IntegrationState(method) for method in ('euler', 'exact')}
        self.odom_publishers = {
            method: self.create_publisher(Odometry, f'/odom_{method}', 10)
            for method in self.states
        }
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.previous = None
        self.previous_publish_ns = None
        self.pending_s = self.pending_theta = 0.0
        self.gt = None
        self.phase = 'waiting'
        self.start_ns = self.stop_ns = None
        self.finished = False
        self.create_log()
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.on_gt, 50)
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info('Waiting for joint states and ground truth before driving.')

    def create_log(self):
        repository = next((p for p in Path(__file__).resolve().parents
                           if (p / 'src_mustafa' / 'package.xml').is_file()), None)
        if repository is None:
            raise RuntimeError('Cannot locate HW2 repository; build with --symlink-install')
        folder = repository / 'Arc Runs'
        folder.mkdir(parents=True, exist_ok=True)
        self.csv_path = folder / f'arc_{uuid4().hex}.csv'
        self.csv_file = self.csv_path.open('x', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'topic', 'sim_time_s', 'stamp_sec', 'stamp_nanosec',
            'frame_id', 'child_frame_id', 'x_m', 'y_m', 'z_m',
            'qx', 'qy', 'qz', 'qw', 'yaw_rad', 'run_time_s', 'phase',
        ])
        self.metadata = dict(self.settings, start_ns=None, stop_ns=None,
                             integration='shared cumulative wheel travel at integration_rate',
                             comparison_frame='arc_start', ground_truth_start=None)
        self.save_metadata()
        self.get_logger().info(f'CSV output: {self.csv_path}')

    def save_metadata(self):
        self.csv_path.with_suffix('.json').write_text(
            json.dumps(self.metadata, indent=2) + '\n', encoding='utf-8')

    def log_pose(self, topic, msg):
        if self.phase == 'waiting' or self.finished:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.writer.writerow([
            topic, self.get_clock().now().nanoseconds / 1e9,
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.header.frame_id, msg.child_frame_id,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w, yaw_of(msg),
            (stamp_ns(msg.header.stamp) - self.start_ns) / 1e9, self.phase,
        ])

    def on_gt(self, msg):
        self.gt = msg
        self.log_pose('/ground_truth/odom', msg)

    def on_joints(self, msg):
        try:
            left = msg.position[msg.name.index(OdometryNode.LEFT_JOINT)]
            right = msg.position[msg.name.index(OdometryNode.RIGHT_JOINT)]
        except (ValueError, IndexError):
            return
        if not (math.isfinite(left) and math.isfinite(right)):
            return
        now = stamp_ns(msg.header.stamp)
        if self.previous is None:
            self.previous = (left, right, now)
            self.previous_publish_ns = now
            return
        old_left, old_right, old_ns = self.previous
        if now <= old_ns:
            return
        if self.phase == 'waiting' or self.finished:
            self.previous = (left, right, now)
            self.previous_publish_ns = now
            return
        if now - old_ns < self.integration_period_ns:
            return
        # Keep the baseline until an integration step is due. Cumulative
        # angles include all wheel motion from the skipped intermediate samples.
        self.previous = (left, right, now)
        dl = (left - old_left) * self.settings['r_left']
        dr = (right - old_right) * self.settings['r_right']
        ds, dtheta = (dl + dr) / 2, (dr - dl) / self.settings['b']
        for state in self.states.values():
            state.update(ds, dtheta)
        self.pending_s += ds
        self.pending_theta += dtheta
        elapsed = now - self.previous_publish_ns
        if elapsed >= self.period_ns:
            self.publish_pair(msg.header.stamp, self.pending_s / (elapsed * 1e-9),
                              self.pending_theta / (elapsed * 1e-9))
            self.previous_publish_ns = now
            self.pending_s = self.pending_theta = 0.0

    def publish_pair(self, stamp, linear, angular):
        for method, state in self.states.items():
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = 'arc_start'
            msg.child_frame_id = f'base_link_{method}'
            msg.pose.pose.position.x = state.x
            msg.pose.pose.position.y = state.y
            msg.pose.pose.orientation.z = math.sin(state.theta / 2)
            msg.pose.pose.orientation.w = math.cos(state.theta / 2)
            msg.twist.twist.linear.x = linear
            msg.twist.twist.angular.z = angular
            self.odom_publishers[method].publish(msg)
            self.log_pose(f'/odom_{method}', msg)
        self.csv_file.flush()

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if self.finished:
            return
        if self.phase == 'waiting':
            if now == 0 or self.previous is None or self.gt is None:
                return
            self.start_ns = now
            self.phase = 'driving'
            self.metadata['start_ns'] = now
            p = self.gt.pose.pose.position
            self.metadata['ground_truth_start'] = [p.x, p.y, yaw_of(self.gt)]
            self.save_metadata()
            self.publish_pair(self.gt.header.stamp, 0.0, 0.0)
            self.log_pose('/ground_truth/odom', self.gt)
        elapsed = (now - self.start_ns) / 1e9
        command = Twist()
        if self.phase == 'driving' and elapsed >= self.settings['duration']:
            self.phase = 'settling'
            self.stop_ns = now
            self.metadata['stop_ns'] = now
            self.save_metadata()
            self.get_logger().info(
                f'{self.settings["duration"]:g}-second arc complete; '
                'stopping and logging settling motion.')
        if self.phase == 'driving':
            command.linear.x = self.settings['v']
            command.angular.z = self.settings['omega']
        self.cmd.publish(command)
        if self.phase == 'settling' and (now - self.stop_ns) / 1e9 >= self.settings['settle_time']:
            self.finished = True
            self.csv_file.flush()
            euler, exact = self.states['euler'], self.states['exact']
            difference = math.hypot(euler.x - exact.x, euler.y - exact.y)
            self.get_logger().info(f'End Euler-Exact separation: {difference:.9f} m. '
                                   'Use plot_arc.py for the matched CSV endpoints and report.')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = Event()
    handlers = {s: signal.signal(s, lambda *_: stop.set())
                for s in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = ArcRun()
        while not stop.is_set() and not node.finished and node.context.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.timer.cancel()
                if node.context.ok():
                    node.cmd.publish(Twist())
                node.csv_file.close()
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
