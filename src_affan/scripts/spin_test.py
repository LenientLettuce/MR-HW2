#!/usr/bin/env python3
"""
AI was used for writing boilerplate code
"""
import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return 2.0 * math.atan2(q.z, q.w)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SpinTest(Node):
    def __init__(self):
        super().__init__('spin_test')
        self.omega = float(self.declare_parameter('omega', 0.5).value)   # commanded rad/s
        self.settle = float(self.declare_parameter('settle', 2.0).value)  # rest time after stop
        self.csv_path = str(self.declare_parameter('csv_path', '').value)
        self.label = str(self.declare_parameter('label', '').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_cb, 10)

        # latest raw yaw from each source 
        self.odom_yaw = None
        self.gt_yaw = None
        # unwrapped total yaw change since the spin started
        self.odom_total = 0.0
        self.gt_total = 0.0

        self.state = 'wait'   
        self.t_stop = None
        self.create_timer(0.05, self.tick) 

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg):
        y = yaw_of(msg)
        if self.state in ('spin', 'settle') and self.odom_yaw is not None:
            self.odom_total += wrap(y - self.odom_yaw)
        self.odom_yaw = y

    def gt_cb(self, msg):
        y = yaw_of(msg)
        if self.state in ('spin', 'settle') and self.gt_yaw is not None:
            self.gt_total += wrap(y - self.gt_yaw)
        self.gt_yaw = y

    def tick(self):
        if self.now() == 0.0:  
            return

        if self.state == 'wait':
            if (self.odom_yaw is not None and self.gt_yaw is not None and self.pub.get_subscription_count() > 0):
                self.state = 'spin'
                self.get_logger().info(f'Both odometries received, spinning at {self.omega} rad/s')
            return

        if self.state == 'spin':
            # stop when /odom says one full revolution has been completed
            if self.odom_total >= 2.0 * math.pi:
                self.pub.publish(Twist())
                self.t_stop = self.now()
                self.state = 'settle'
                return
            cmd = Twist()
            cmd.angular.z = self.omega   # v = 0
            self.pub.publish(cmd)

        elif self.state == 'settle':
            self.pub.publish(Twist())     # keep commanding zero while the robot comes to rest
            if self.now() - self.t_stop >= self.settle:
                self.report()
                self.state = 'done'
                raise SystemExit

    def report(self):
        ratio = self.odom_total / self.gt_total if self.gt_total else float('nan')
        print(f'odom yaw change : {math.degrees(self.odom_total):9.3f} deg')
        print(f'truth yaw change: {math.degrees(self.gt_total):9.3f} deg')
        print(f'ratio odom/truth: {ratio:9.5f}')
        if self.csv_path:
            p = Path(self.csv_path)
            new = not p.exists()
            with p.open('a', newline='') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(['label', 'omega', 'odom_deg', 'truth_deg', 'ratio'])
                w.writerow([self.label, self.omega,
                            f'{math.degrees(self.odom_total):.4f}',
                            f'{math.degrees(self.gt_total):.4f}', f'{ratio:.6f}'])


def main():
    rclpy.init()
    node = SpinTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
