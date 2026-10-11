"""Convenience wrapper for sim.launch.py with own odometry enabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('a2_description')
    return LaunchDescription([
        DeclareLaunchArgument('pair', default_value='P08'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('method', default_value='rk2', choices=['euler', 'rk2', 'exact']),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'odometry.yaml'),
            description='Odometry parameter YAML; dimensions are set here, not by pair',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'sim.launch.py')
            ),
            launch_arguments={
                'pair': LaunchConfiguration('pair'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'plugin_tf': 'false',
                'own_odometry': 'true',
                'odometry_params_file': LaunchConfiguration('params_file'),
                'method': LaunchConfiguration('method'),
            }.items(),
        ),
    ])
