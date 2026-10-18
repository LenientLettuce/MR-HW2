#!/usr/bin/env python3
"""Load CSV waypoints for a pure pursuit controller."""

import csv
import json
import math
import os
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data


def pose_of(msg):
    """Return x, y, yaw from an Odometry message (yaw in radians)."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw


def cross_track_error(x, y, waypoints):
    """Unsigned distance to the reference polyline, including its corners."""
    best = float('inf')
    for a, b in zip(waypoints, waypoints[1:] + waypoints[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_squared = dx * dx + dy * dy
        t = (max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy) / length_squared))
             if length_squared else 0.0)
        best = min(best, math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
    return best


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.look_ahead = self.declare_parameter('Ld', 0.3).value
        self.v = self.declare_parameter('v', 0.2).value
        if not math.isfinite(self.look_ahead) or self.look_ahead <= 0.0:
            raise ValueError('Ld must be finite and greater than zero')
        if not math.isfinite(self.v) or self.v < 0.0:
            raise ValueError('v must be finite and nonnegative')
        
        #loading waypoints
        path = self.declare_parameter('path', 'square').value
        if path not in ('circle', 'square'):
            raise ValueError("path must be 'circle' or 'square'")
        filename = os.path.join(
            get_package_share_directory('a2_description'), 'paths', f'{path}.csv')
        with open(filename, newline='') as stream:
            self.waypoints = [
                (float(row['x']), float(row['y']), float(row['yaw']))
                for row in csv.DictReader(stream)
            ]
            
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Remove the repeated endpoint only for measuring progress around the loop.
        self.loop_points = self.waypoints[:]
        if self.loop_points[-1][:2] == self.loop_points[0][:2]:
            self.loop_points.pop()
        self.path_distances = [0.0]
        for a, b in zip(self.loop_points, self.loop_points[1:]):
            self.path_distances.append(self.path_distances[-1] + math.hypot(b[0]-a[0], b[1]-a[1]))
        a, b = self.loop_points[-1], self.loop_points[0]
        self.path_length = self.path_distances[-1] + math.hypot(b[0]-a[0], b[1]-a[1])
        self.previous_path_distance = None
        self.loop_progress = 0.0
        self.finished = False
        self.stop_time = None
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filename}')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = None
        self.command_v = self.command_omega = 0.0
        self.spawn_x = self.declare_parameter('spawn_x', 0.0).value
        self.spawn_y = self.declare_parameter('spawn_y', 0.0).value
        self.spawn_yaw = self.declare_parameter('spawn_yaw', 0.0).value
        pair = self.declare_parameter('pair', 'REF').value
        own_odometry = self.declare_parameter('own_odometry', False).value
        repository = next((p for p in Path(__file__).resolve().parents
                           if (p / 'src_mustafa' / 'package.xml').is_file()), None)
        default_dir = (repository or Path.cwd()) / 'Pure Pursuit Runs'
        folder = Path(self.declare_parameter('runs_dir', str(default_dir)).value).expanduser()
        folder.mkdir(parents=True, exist_ok=True)
        run_number = 1
        while True:
            self.csv_path = folder / f'{path}_run{run_number}.csv'
            try:
                self.csv_file = self.csv_path.open('x', newline='', encoding='utf-8')
                break
            except FileExistsError:
                run_number += 1
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'topic', 'sim_time_s', 'run_time_s', 'frame_id',
            'raw_x_m', 'raw_y_m', 'raw_yaw_rad',
            'x_m', 'y_m', 'yaw_rad', 'cross_track_error_m',
            'command_v_m_s', 'command_omega_rad_s',
        ])
        self.csv_file.flush()
        self.csv_path.with_suffix('.json').write_text(json.dumps({
            'path': path, 'pair': pair, 'Ld_m': self.look_ahead, 'v_m_s': self.v,
            'own_odometry': own_odometry,
            'spawn_pose_world': [self.spawn_x, self.spawn_y, self.spawn_yaw],
            'comparison_frame': 'odom',
            'cross_track_error': 'unsigned shortest distance to reference polyline',
            'time_origin': 'first recorded pose timestamp',
            'completion': 'one odometry lap, measured by nearest-waypoint progress',
            'reference_waypoints': self.waypoints,
            'command_columns': 'on odom rows only; requested commands, not measured velocities',
        }, indent=2) + '\n', encoding='utf-8')
        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.on_ground_truth, qos_profile_sensor_data)
        self.get_logger().info(f'Recording run to {self.csv_path}')

    def on_odom(self, msg):
        self.x, self.y, self.theta = pose_of(msg)
        if not self.finished and self.loop_complete():
            self.finished = True
            self.stop_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            metadata_path = self.csv_path.with_suffix('.json')
            metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
            metadata['stop_sim_time_s'] = self.stop_time
            metadata['stop_run_time_s'] = (
                self.stop_time - self.start_time if self.start_time is not None else 0.0)
            metadata_path.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
            self.get_logger().info('One loop complete according to /odom; stopping.')
        if self.finished:
            self.publish_command(Twist())
        else:
            self.look_forward()
        self.log_pose('/odom', msg)

    def loop_complete(self):
        """Unwrap nearest-waypoint progress so returning to zero counts as a lap.

        This measures progress along the reference, not distance driven, so
        corner cutting does not require driving the full square perimeter.
        """
        nearest = min(range(len(self.loop_points)), key=lambda i:
                      (self.loop_points[i][0]-self.x)**2 + (self.loop_points[i][1]-self.y)**2)
        distance = self.path_distances[nearest]
        if self.previous_path_distance is not None:
            delta = distance - self.previous_path_distance
            delta = (delta + self.path_length / 2.0) % self.path_length - self.path_length / 2.0
            self.loop_progress += delta
        self.previous_path_distance = distance
        # Allow one waypoint's spacing (about 1 cm) at the finish.
        return self.loop_progress >= self.path_length - 0.015

    def on_ground_truth(self, msg):
        self.log_pose('/ground_truth/odom', msg)

    def log_pose(self, topic, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Keep final braking/settling samples, then stop extending the run CSV.
        if self.stop_time is not None and timestamp > self.stop_time + 1.0:
            return
        raw_x, raw_y, raw_yaw = pose_of(msg)
        x, y, yaw = raw_x, raw_y, raw_yaw
        if topic == '/ground_truth/odom':
            # Ground truth is in world; CSV references are in the spawn's odom frame.
            dx, dy = x - self.spawn_x, y - self.spawn_y
            c, s = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
            x, y = c * dx + s * dy, -s * dx + c * dy
            yaw = math.atan2(math.sin(yaw-self.spawn_yaw), math.cos(yaw-self.spawn_yaw))
        if self.start_time is None:
            self.start_time = timestamp
        self.writer.writerow([
            topic, timestamp, timestamp - self.start_time, msg.header.frame_id,
            raw_x, raw_y, raw_yaw, x, y, yaw,
            cross_track_error(x, y, self.waypoints),
            self.command_v if topic == '/odom' else '',
            self.command_omega if topic == '/odom' else '',
        ])
        self.csv_file.flush()

    def publish_command(self, cmd):
        self.command_v, self.command_omega = cmd.linear.x, cmd.angular.z
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        self.csv_file.close()
        return super().destroy_node()

    def look_forward(self):
        #we make sure that we are only looking forward
        ahead_indices = [
            i for i, (wx, wy, _) in enumerate(self.waypoints)
            if (
                math.cos(self.theta) * (wx - self.x)
                + math.sin(self.theta) * (wy - self.y)
            ) > 0.0
        ]

        #then we find the closest waypoint
        closest_idx = min(
            ahead_indices,
            key=lambda i: (
                (self.waypoints[i][0] - self.x)**2
                + (self.waypoints[i][1] - self.y)**2
            ),
            default=None,
        )

        if closest_idx is None:
            self.publish_command(Twist())
            return

        # Search in CSV order from the closest point, wrapping for the loop.
        # Choose the first ahead point at least Ld metres from the robot.
        target = None
        for offset in range(len(self.waypoints)):
            i = (closest_idx + offset) % len(self.waypoints)
            wx, wy, _ = self.waypoints[i]
            dx, dy = wx - self.x, wy - self.y
            forward = math.cos(self.theta) * dx + math.sin(self.theta) * dy
            if forward > 0.0 and dx**2 + dy**2 >= self.look_ahead**2:
                target = (wx, wy)
                break

        if target is None:
            self.publish_command(Twist())
            return

        wx, wy = target
        
        dx = wx - self.x
        dy = wy - self.y

        x_robot =  math.cos(self.theta) * dx + math.sin(self.theta) * dy
        y_robot = -math.sin(self.theta) * dx + math.cos(self.theta) * dy
        
        distance_squared = x_robot**2 + y_robot**2
        if distance_squared <= 1e-12:
            self.publish_command(Twist())
            return
        arc_curvature = 2 * y_robot / distance_squared
        omega = max(-1.5, min(1.5, self.v * arc_curvature))
        
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = omega
        self.publish_command(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PurePursuit()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
