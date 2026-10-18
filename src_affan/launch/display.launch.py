"""display.launch.py - inspect the robot model in RViz, without Gazebo.

    ros2 launch a2_description display.launch.py pair:=P03

Opens RViz with a slider window (joint_state_publisher_gui) for the two
wheel joints. Drag a slider and watch that wheel's frame rotate in the TF
display: this is the quickest way to see what a joint's origin and axis
actually do. No physics runs here, so the robot cannot drive.
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'a2_description'


def normalise_pair(raw):
    raw = raw.strip().upper()
    if raw == 'REF':
        return raw
    digits = raw[1:] if raw.startswith('P') else raw
    return f'P{int(digits):02d}'


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory(PKG)
    pair = normalise_pair(LaunchConfiguration('pair').perform(context))
    with open(os.path.join(share, 'config', 'pairs.yaml')) as f:
        doc = yaml.safe_load(f)
    actual = doc['pairs'][pair]['actual']

    robot_description = xacro.process_file(
        os.path.join(share, 'urdf', 'a2_bot.urdf.xacro'),
        mappings={
            'wheel_radius_left': str(actual['wheel_radius_left']),
            'wheel_radius_right': str(actual['wheel_radius_right']),
            'wheel_separation': str(actual['wheel_separation']),
            'wheel_width': str(doc['common']['wheel_width']),
        },
    ).toxml()

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'config', 'display.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pair', default_value='REF',
                              description='Your pair ID (P01..P12) or REF'),
        OpaqueFunction(function=launch_setup),
    ])
