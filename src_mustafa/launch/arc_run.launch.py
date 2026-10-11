"""Run the Euler/Exact arc comparison; launch simulation separately first."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'v': '0.2', 'omega': '0.2', 'duration': '30.0',
                'settle_time': '1.0', 'publish_rate': '50.0', 'integration_rate': '100.0',
                'r_left': '0.054146', 'r_right': '0.053854', 'b': '0.319'}
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        Node(
            package='a2_description', executable='arc_run.py', output='screen',
            parameters=[{'use_sim_time': True, **{
                k: ParameterValue(LaunchConfiguration(k), value_type=float)
                for k in defaults
            }}],
        ),
    ])
