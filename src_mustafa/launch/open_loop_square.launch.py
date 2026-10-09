"""Start the student's square node with simulation time enabled.

Launch sim.launch.py in another terminal first.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a2_description',
            executable='open_loop_square.py',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
