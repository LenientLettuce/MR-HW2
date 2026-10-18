#!/usr/bin/env python3
"""Own wheel odometry"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    LEFT_JOINT = 'left_wheel_joint'
    RIGHT_JOINT = 'right_wheel_joint'

    def __init__(self):
        super().__init__('odometry_node')

        # Defaults match P08 nameplate dimensions (0.053, 0.053, 0.313) for Question 5.
        self.r_left = self.declare_parameter('r_left', 0.053).value
        self.r_right = self.declare_parameter('r_right', 0.053).value
        self.b = self.declare_parameter('b', 0.313).value
        self.method = self.declare_parameter('method', 'rk2').value
        self.publish_rate = self.declare_parameter('publish_rate', 50.0).value
        if not math.isfinite(self.publish_rate) or not 0.0 < self.publish_rate <= 1e9:
            raise ValueError('publish_rate must be finite and between 0 and 1e9 Hz')
        self.publish_period_ns = max(1, round(1e9 / self.publish_rate))

        # Supported integration methods.
        if self.method not in ('euler', 'exact', 'rk2'):
            raise ValueError('method must be euler, exact or rk2')
        if self.r_left <= 0 or self.r_right <= 0 or self.b <= 0:
            raise ValueError('r_left, r_right and b must be positive')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.previous_left_angle = None
        self.previous_right_angle = None
        self.previous_stamp = None
        self.previous_publish_stamp_ns = None
        self.distance_since_publish = 0.0
        self.heading_since_publish = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom_own', 10)
        # Enable only when the launch file moves plugin odometry off /odom.
        publish_odom = self.declare_parameter('publish_odom', False).value
        self.odom_alias_pub = (
            self.create_publisher(Odometry, '/odom', 10) if publish_odom else None
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Wheel odometry ready: method={self.method}, '
            f'r_left={self.r_left}, r_right={self.r_right}, b={self.b}, '
            f'publish_rate={self.publish_rate} Hz'
        )

    def joint_states_callback(self, msg: JointState):
        """Integrate cumulative wheel-angle changes and publish odometry/TF."""
        try:
            left_index = msg.name.index(self.LEFT_JOINT)
            right_index = msg.name.index(self.RIGHT_JOINT)
        except ValueError:
            self.get_logger().warning(
                'JointState is missing a wheel joint; skipping message.',
                throttle_duration_sec=5.0,
            )
            return

        if max(left_index, right_index) >= len(msg.position):
            self.get_logger().warning(
                'JointState is missing a wheel position; skipping message.',
                throttle_duration_sec=5.0,
            )
            return
        left_angle = msg.position[left_index]
        right_angle = msg.position[right_index]
        if not (math.isfinite(left_angle) and math.isfinite(right_angle)):
            self.get_logger().warning(
                'JointState contains a nonfinite wheel angle; skipping message.',
                throttle_duration_sec=5.0,
            )
            return

        if self.previous_left_angle is None or self.previous_right_angle is None:
            self.previous_left_angle = left_angle
            self.previous_right_angle = right_angle
            self.previous_stamp = msg.header.stamp
            self.previous_publish_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            )
            return

        dt = (
            msg.header.stamp.sec - self.previous_stamp.sec
            + (msg.header.stamp.nanosec - self.previous_stamp.nanosec) * 1e-9
        )
        if dt <= 0.0:
            return

        left_travel = (left_angle - self.previous_left_angle) * self.r_left
        right_travel = (right_angle - self.previous_right_angle) * self.r_right

        delta_s = (right_travel + left_travel) * 0.5
        delta_theta = (right_travel - left_travel) / self.b
        self.integrate_pose(delta_s, delta_theta)

        self.linearVel = delta_s / dt
        self.angularVel = delta_theta / dt

        self.previous_left_angle = left_angle
        self.previous_right_angle = right_angle
        self.previous_stamp = msg.header.stamp

        self.distance_since_publish += delta_s
        self.heading_since_publish += delta_theta
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        publish_elapsed_ns = stamp_ns - self.previous_publish_stamp_ns
        if publish_elapsed_ns < self.publish_period_ns:
            return

        publish_dt = publish_elapsed_ns * 1e-9
        self.publish_odometry(
            msg.header.stamp,
            self.distance_since_publish / publish_dt,
            self.heading_since_publish / publish_dt,
        )
        self.previous_publish_stamp_ns = stamp_ns
        self.distance_since_publish = 0.0
        self.heading_since_publish = 0.0

    def integrate_pose(self, delta_s: float, delta_theta: float):
        """Update self.x, self.y and self.theta with the selected method."""
        if self.method == 'euler':
            self.x += delta_s * math.cos(self.theta)
            self.y += delta_s * math.sin(self.theta)
            self.theta += delta_theta
        elif self.method == 'exact':
            headingOld = self.theta
            ratio = delta_s / delta_theta
            self.theta += delta_theta
            self.x += ratio * (math.sin(self.theta) - math.sin(headingOld))
            self.y += -ratio * (math.cos(self.theta) - math.cos(headingOld))
        else:
            direction = self.theta + delta_theta * 0.5

            self.x += delta_s * math.cos(direction)
            self.y += delta_s * math.sin(direction)
            self.theta += delta_theta

    def publish_odometry(self, stamp, linear_velocity: float,
                         angular_velocity: float):
        """Publish the current pose and its matching odom -> base_link TF."""
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom)
        if self.odom_alias_pub is not None:
            self.odom_alias_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
