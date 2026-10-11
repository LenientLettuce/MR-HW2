"""Start the student's square node with simulation time enabled.

Launch sim.launch.py in another terminal first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        Node(
            package='a2_description',
            executable='open_loop_square.py',
            output='screen',
            parameters=[{'use_sim_time': True, 'odom_topic': LaunchConfiguration('odom_topic')}],
        ),
    ])
