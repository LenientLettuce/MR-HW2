#!/usr/bin/env python3
"""umbmark_driver.py - drive one UMBmark square, closed loop on YOUR odometry.

    ros2 run a2_description umbmark_driver.py --ros-args -p use_sim_time:=true \
        -p odom_topic:=/odom_own -p direction:=cw -p side:=2.0 -p csv:=umbmark.csv

The robot drives a square of side `side`, steering only by the odometry on
`odom_topic`: it holds its heading along each leg, stops when odometry says
it has travelled `side` metres, and turns until odometry says it has turned
90 degrees. That is exactly how UMBmark works on a real robot: the robot
believes it has returned to its start.

At the end it compares where the robot REALLY is (/ground_truth/odom, your
tape measure) with where odometry says it is, both expressed in the start
frame, and appends one line to `csv`:

    direction, eps_x, eps_y, eps_theta      (true - odometry; m, m, rad)

Run it 5 times clockwise and 5 times counter-clockwise, relaunching the
simulation before each run.
"""
import csv
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return 2.0 * math.atan2(q.z, q.w)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_of(msg):
    p = msg.pose.pose.position
    return (p.x, p.y, yaw_of(msg))


def relative(p0, p):
    """Pose p expressed in the frame of pose p0."""
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - p0[2]))


class UMBmarkDriver(Node):
    def __init__(self):
        super().__init__('umbmark_driver')
        self.side = self.declare_parameter('side', 2.0).value
        self.v = self.declare_parameter('v', 0.15).value
        self.w_max = self.declare_parameter('omega', 0.4).value
        self.sign = 1.0 if self.declare_parameter('direction', 'ccw').value == 'ccw' else -1.0
        self.csv_path = self.declare_parameter('csv', 'umbmark.csv').value
        odom_topic = self.declare_parameter('odom_topic', '/odom_own').value

        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        self.odom = self.gt = None
        self.odom0 = self.gt0 = None
        self.leg = 0                  # 0..3
        self.phase = 'start'          # start -> straight -> turn -> ... -> settle -> done
        self.create_timer(0.02, self.tick)

    def on_odom(self, msg):
        self.odom = pose_of(msg)

    def on_gt(self, msg):
        self.gt = pose_of(msg)

    def heading_target(self):
        return wrap(self.odom0[2] + self.sign * self.leg * math.pi / 2.0)

    def tick(self):
        cmd = Twist()
        if self.odom is None or self.gt is None:
            return
        if self.phase == 'start':
            self.odom0, self.gt0 = self.odom, self.gt
            self.leg_start = self.odom
            self.phase = 'straight'

        if self.phase == 'straight':
            th = self.heading_target()
            along = ((self.odom[0] - self.leg_start[0]) * math.cos(th)
                     + (self.odom[1] - self.leg_start[1]) * math.sin(th))
            if along >= self.side:
                self.phase = 'turn'
                self.leg += 1
            else:
                cmd.linear.x = self.v
                cmd.angular.z = 2.0 * wrap(th - self.odom[2])   # hold heading

        elif self.phase == 'turn':
            err = wrap(self.heading_target() - self.odom[2])
            if abs(err) < 0.002:
                if self.leg == 4:
                    self.phase = 'settle'
                    self.settle_start = self.get_clock().now()
                else:
                    self.phase = 'straight'
                    self.leg_start = self.odom
            else:
                cmd.angular.z = max(-self.w_max, min(self.w_max, 3.0 * err))
                if abs(cmd.angular.z) < 0.05:
                    cmd.angular.z = math.copysign(0.05, err)

        elif self.phase == 'settle':
            if (self.get_clock().now() - self.settle_start).nanoseconds > 2e9:
                self.finish()
                self.phase = 'done'

        self.cmd.publish(cmd)
        if self.phase == 'done':
            raise SystemExit

    def finish(self):
        true_rel = relative(self.gt0, self.gt)
        odom_rel = relative(self.odom0, self.odom)
        eps = (true_rel[0] - odom_rel[0], true_rel[1] - odom_rel[1],
               wrap(true_rel[2] - odom_rel[2]))
        direction = 'ccw' if self.sign > 0 else 'cw'
        new = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['direction', 'eps_x', 'eps_y', 'eps_theta'])
            w.writerow([direction, f'{eps[0]:.5f}', f'{eps[1]:.5f}', f'{eps[2]:.6f}'])
        self.get_logger().info(
            f'{direction}: eps_x = {100 * eps[0]:.2f} cm, eps_y = {100 * eps[1]:.2f} cm, '
            f'eps_theta = {math.degrees(eps[2]):.2f} deg  -> appended to {self.csv_path}')


def main():
    rclpy.init()
    node = UMBmarkDriver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.cmd.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
