#!/usr/bin/env python3
"""path_publisher.py - show a reference path in RViz.

Started for you by the launch file:

    ros2 launch a2_description sim.launch.py pair:=P03 path:=square

or on its own, with a simulation already running:

    ros2 run a2_description path_publisher.py --ros-args -p use_sim_time:=true \
        -p path:=circle

`path` is `circle` or `square` (the files in the package's paths/ folder), or
the full path to any CSV with columns x, y, yaw. The path is published once,
latched, as a nav_msgs/Path on /path in the `odom` frame, and RViz draws it
in yellow.

This node is for display only. Your controller should read the waypoints
straight from the CSV file, e.g.

    import numpy as np, os
    from ament_index_python.packages import get_package_share_directory
    f = os.path.join(get_package_share_directory('a2_description'), 'paths', 'square.csv')
    xy_yaw = np.loadtxt(f, delimiter=',', skiprows=1)      # columns x, y, yaw
"""
import csv
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def resolve(path):
    """'circle' / 'square' -> the packaged CSV; anything else is a file name."""
    if path in ('circle', 'square'):
        return os.path.join(get_package_share_directory('a2_description'),
                            'paths', f'{path}.csv')
    return os.path.expanduser(path)


def load(filename):
    with open(filename, newline='') as f:
        return [(float(r['x']), float(r['y']), float(r['yaw'])) for r in csv.DictReader(f)]


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        filename = resolve(self.declare_parameter('path', 'square').value)
        frame = self.declare_parameter('frame', 'odom').value
        pts = load(filename)

        msg = Path()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(ps)

        # Latched: RViz (or anything else) receives it whenever it subscribes.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Path, '/path', qos)
        self.pub.publish(msg)
        self.get_logger().info(
            f'{len(pts)} points from {filename} on /path (frame "{frame}")')


def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
