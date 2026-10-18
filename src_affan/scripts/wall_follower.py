#!/usr/bin/env python3
"""
Note: AI was used for understanding some stuff to write the node i.e. QoS, CSV logging, boilerplate for node.
"""

import csv
import math
from pathlib import Path
from uuid import uuid4

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.Kp = float(self.declare_parameter('Kp', 2.5).value)
        self.Kd = float(self.declare_parameter('Kd', 0.0).value)
        self.Ki = float(self.declare_parameter('Ki', 0.0).value)
        self.v = float(self.declare_parameter('v', 0.2).value)
        self.max_omega = float(self.declare_parameter('max_omega', 1.5).value)
        self.D_ref = float(self.declare_parameter('D_ref', 0.5).value)
        self.theta_deg = float(self.declare_parameter('theta_deg', 45.0).value)
        self.lookahead_L = float(self.declare_parameter('lookahead_L', 0.0).value)
        self.csv_path_param = str(self.declare_parameter('csv_path', '').value)
        self.duration = float(self.declare_parameter('duration', 0.0).value)
        self.derivative_mode = str(
            self.declare_parameter('derivative_mode', 'diff').value
        ).lower()  # 'diff' is model free and 'alpha' is model-based
    

        self.theta = math.radians(self.theta_deg)

        # beam b is 270 deg (straight right), beam a is theta_deg further forward
        self.idx_b = 270
        self.idx_a = int(round((270 + self.theta_deg) % 360))

        # State variables
        self.start_time = None
        self.prev_time = None
        self.prev_error = None
        self.integral = 0.0
        self.last_omega = 0.0

        # Ground truth pose tracking
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_yaw = 0.0

        # Setup CSV log
        self.setup_csv()

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # QoS profile compatible with sensor data
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos
        )

        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.gt_callback, 10
        )

        self.get_logger().info(
            f'Wall Follower initialized: Kp={self.Kp:.2f}, Kd={self.Kd:.2f}, Ki={self.Ki:.2f}, '
            f'D_ref={self.D_ref:.2f} m, L={self.lookahead_L:.2f} m, '
            f'theta={self.theta_deg:.1f} deg (beams: b={self.idx_b}, a={self.idx_a}), '
            f'derivative_mode={self.derivative_mode}'
        )

    def setup_csv(self):
        """Prepare CSV file for logging."""
        if self.csv_path_param:
            self.csv_path = Path(self.csv_path_param).resolve()
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        else:
            # Default to 'Wall Follower Runs' in repo root
            repo_root = Path(__file__).resolve().parents[2]
            runs_dir = repo_root / 'Wall Follower Runs'
            runs_dir.mkdir(parents=True, exist_ok=True)
            self.csv_path = runs_dir / f'wall_follower_{uuid4().hex[:8]}.csv'

        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        # Header includes all required fields from Problem 7(a)
        self.csv_writer.writerow([
            'time', 'D', 'alpha', 'e', 'omega',
            'edot', 'a_range', 'b_range', 'gt_x', 'gt_y', 'gt_yaw'
        ])
        self.csv_file.flush()
        self.get_logger().info(f'Logging to CSV: {self.csv_path}')

    def log_row(self, t, D, alpha, e, omega, e_dot, a, b):
        """Write one CSV row. inf / nan are written as 'inf' / 'nan' (numpy reads them)."""
        self.csv_writer.writerow([
            f'{t:.4f}',
            f'{D:.5f}',
            f'{alpha:.5f}',
            f'{e:.5f}',
            f'{omega:.5f}',
            f'{e_dot:.5f}',
            f'{a:.5f}',
            f'{b:.5f}',
            f'{self.gt_x:.4f}',
            f'{self.gt_y:.4f}',
            f'{self.gt_yaw:.4f}',
        ])
        self.csv_file.flush()

    def gt_callback(self, msg: Odometry):
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.gt_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


    def scan_callback(self, msg: LaserScan):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec == 0.0:
            return

        if self.start_time is None:
            # The first scan only starts the clock.
            self.start_time = now_sec
            self.prev_time = now_sec
            return

        t_elapsed = now_sec - self.start_time

        # Check duration limit
        if self.duration > 0.0 and t_elapsed >= self.duration:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'Target duration {self.duration} s reached. Stopping.')
            self.csv_file.close()
            raise SystemExit

        dt = now_sec - self.prev_time
        if dt <= 0.0:
            return 

        # Extract beam ranges directly from /scan
        b = msg.ranges[self.idx_b]
        a = msg.ranges[self.idx_a]


        # alpha = atan2(a*cos(theta) - b, a*sin(theta))
        alpha = math.atan2(
            a * math.cos(self.theta) - b,
            a * math.sin(self.theta)
        )
        D = b * math.cos(alpha)

        # e = (D_ref - D) - L * sin(alpha)
        # When L = 0, this is pure distance error: e = D_ref - D
        # D > D_ref (too far from the right wall): e < 0 -> omega < 0 (turn right, towards the wall)
        # D < D_ref (too close):                   e > 0 -> omega > 0 (turn left, away from the wall)
        e = (self.D_ref - D) - self.lookahead_L * math.sin(alpha)

        # Derivative calculation
        if self.derivative_mode == 'alpha':
            # Kinematic derivative: dD/dt = v*sin(alpha), so e_dot = -v*sin(alpha) (exact for L = 0)
            e_dot = -self.v * math.sin(alpha)
        else:
            # Model-free finite difference: (e_k - e_{k-1}) / dt
            if self.prev_error is None:
                e_dot = 0.0
            else:
                e_dot = (e - self.prev_error) / dt

        # Integral calculation
        self.integral += e * dt

        # PID control law
        omega = self.Kp * e + self.Kd * e_dot + self.Ki * self.integral


        # Saturate omega (|omega| <= max_omega) and prevent sending NaN to Gazebo
        if math.isnan(omega):
            omega = -self.max_omega
        else:
            omega = max(-self.max_omega, min(self.max_omega, omega))
        self.last_omega = omega

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        self.log_row(t_elapsed, D, alpha, e, omega, e_dot, a, b)

        # Update previous state
        self.prev_time = now_sec
        self.prev_error = e

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()