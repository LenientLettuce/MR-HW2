#!/usr/bin/env python3
"""cone_driver.py - move the cone along a scripted path.

    ros2 launch a2_description sim.launch.py cone:=true      # first
    ros2 run a2_description cone_driver.py --ros-args -p use_sim_time:=true \
        -p profile:=line -p speed:=0.2

Profiles (the cone starts facing the same way as the robot):
    line     straight ahead at `speed`
    stopgo   `speed` for `go` seconds, then stopped for `stop` seconds, repeated
    arc      `speed` along a circle of radius `radius`, turning left

The cone drives for `duration` seconds of simulated time, then stops. From
the default spawn, 15 s at 0.2 m/s takes it 3 m, well short of the wall.

To steer the cone by hand instead:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=/cone/cmd_vel
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ConeDriver(Node):
    def __init__(self):
        super().__init__('cone_driver')
        self.profile = self.declare_parameter('profile', 'line').value
        self.speed = float(self.declare_parameter('speed', 0.2).value)
        self.duration = float(self.declare_parameter('duration', 15.0).value)
        self.radius = float(self.declare_parameter('radius', 2.0).value)
        self.go = float(self.declare_parameter('go', 3.0).value)
        self.stop = float(self.declare_parameter('stop', 3.0).value)
        if self.profile not in ('line', 'stopgo', 'arc'):
            raise ValueError(f"profile must be line, stopgo or arc, not {self.profile!r}")

        self.pub = self.create_publisher(Twist, '/cone/cmd_vel', 10)
        self.t0 = None
        self.done = False
        self.create_timer(0.05, self.tick)            # 20 Hz, simulated time

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:                                 # /clock not received yet
            return
        if self.pub.get_subscription_count() == 0:     # bridge not connected yet:
            return                                     # commands would be lost
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info(
                f'cone: {self.profile} at {self.speed} m/s for {self.duration} s')
        t = now - self.t0

        cmd = Twist()
        if t < self.duration:
            moving = True
            if self.profile == 'stopgo':
                moving = (t % (self.go + self.stop)) < self.go
            if moving:
                cmd.linear.x = self.speed
                if self.profile == 'arc':
                    cmd.angular.z = self.speed / self.radius
        elif not self.done:
            self.done = True
            self.get_logger().info('cone: done, stopped')
        self.pub.publish(cmd)                          # keeps publishing zero once done


def main():
    rclpy.init()
    node = ConeDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
