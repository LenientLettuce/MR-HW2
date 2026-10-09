"""sim.launch.py - start the A2 simulation.

    ros2 launch a2_description sim.launch.py pair:=P03

Starts Gazebo Sim with the arena world, spawns YOUR pair's robot, starts
robot_state_publisher, bridges topics between Gazebo and ROS 2, and opens
RViz.

Arguments (all optional):
    pair                     P01..P12, or REF        (default REF)
    plugin_separation_scale  scales ONLY the diff-drive plugin's
                             wheel_separation        (default 1.0)
    lidar_noise              standard deviation (m) of the LiDAR range
                             noise; 0 turns it off   (default 0.01)
    plugin_tf                bridge the plugin's odom->base_link TF
                                                     (default true)
    gt_tf                    bridge the ground-truth world->base_link_gt TF
                                                     (default true)
    gui                      show the Gazebo window  (default true)
    rviz                     open RViz               (default true)
    path                     circle or square: show that reference path in
                             RViz (on /path); empty for none  (default '')
    x, y, yaw                spawn pose in the world frame (m, m, rad)
                                                     (default 0, 0, 0)
    cone                     add the movable cone 1.5 m ahead of the robot
                                                     (default false)
"""
import math
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'a2_description'


def normalise_pair(raw):
    """Accept 'P03', 'p03', '3' or '03'; return 'P03' (or 'REF')."""
    raw = raw.strip().upper()
    if raw == 'REF':
        return raw
    digits = raw[1:] if raw.startswith('P') else raw
    return f'P{int(digits):02d}'


def load_pair(share, pair):
    with open(os.path.join(share, 'config', 'pairs.yaml')) as f:
        doc = yaml.safe_load(f)
    if pair not in doc['pairs']:
        raise RuntimeError(
            f"pair '{pair}' not in config/pairs.yaml "
            f"(choose from {', '.join(doc['pairs'])})")
    return doc['pairs'][pair]['actual'], doc['common']['wheel_width']


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory(PKG)
    pair = normalise_pair(LaunchConfiguration('pair').perform(context))
    scale = float(LaunchConfiguration('plugin_separation_scale').perform(context))
    lidar_noise = float(LaunchConfiguration('lidar_noise').perform(context))
    if lidar_noise < 0.0:
        raise RuntimeError(f'lidar_noise must be >= 0, got {lidar_noise:g}')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    x0 = float(LaunchConfiguration('x').perform(context))
    y0 = float(LaunchConfiguration('y').perform(context))
    yaw0 = float(LaunchConfiguration('yaw').perform(context))
    if max(abs(x0), abs(y0)) > 5.5:
        raise RuntimeError(f'spawn ({x0:g}, {y0:g}) is outside the arena (|x|, |y| <= 5.5 m)')
    cone = LaunchConfiguration('cone').perform(context).lower() == 'true'

    actual, wheel_width = load_pair(share, pair)
    r_l = actual['wheel_radius_left']
    r_r = actual['wheel_radius_right']
    b = actual['wheel_separation']
    r_mean = (r_l + r_r) / 2.0

    # Expand the xacro into a URDF string, with this pair's dimensions.
    robot_description = xacro.process_file(
        os.path.join(share, 'urdf', 'a2_bot.urdf.xacro'),
        mappings={
            'wheel_radius_left': str(r_l),
            'wheel_radius_right': str(r_r),
            'wheel_separation': str(b),
            'wheel_width': str(wheel_width),
            'plugin_separation_scale': str(scale),
            'lidar_noise': str(lidar_noise),
        },
    ).toxml()

    world = os.path.join(share, 'worlds', 'a2_arena.sdf')
    # -r: start running immediately.  -s --headless-rendering: no window,
    # but rendering sensors (the LiDAR) still work.
    # Quote the world path because workspace directories may contain spaces.
    gz_args = f'-r -v 2 "{world}"' if gui else f'-r -s --headless-rendering -v 2 "{world}"'

    if os.name == 'nt':
        # Windows (RoboStack/pixi): server and GUI in one `gz sim` process
        # crashes, so run them separately. The server always renders the
        # LiDAR headless. `gz` is a .bat, hence shell=True.
        gazebo = [ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', '-v', '2', '--headless-rendering', f'"{world}"'],
            shell=True, output='screen',
            on_exit=Shutdown(reason='Gazebo server exited'))]
        if gui:
            gazebo.append(ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2'],
                                         shell=True, output='screen'))
    else:
        gazebo = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        )]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn from the /robot_description topic. base_link is r_mean above
    # the ground; drop the robot from 2 mm higher so it settles cleanly.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'a2_bot',
                   '-x', f'{x0}', '-y', f'{y0}', '-z', f'{r_mean + 0.002:.4f}',
                   '-Y', f'{yaw0}'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )

    bridge_plugin_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_plugin_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('plugin_tf')),
        parameters=[{'config_file': os.path.join(share, 'config', 'bridge_plugin_tf.yaml'),
                     'use_sim_time': True}],
    )

    bridge_gt_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ground_truth_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gt_tf')),
        parameters=[{'config_file': os.path.join(share, 'config', 'bridge_ground_truth_tf.yaml'),
                     'use_sim_time': True}],
    )

    # world -> odom is the spawn pose: odometry starts at zero wherever the
    # robot is spawned. With the default spawn it is the identity.
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['--x', f'{x0}', '--y', f'{y0}', '--yaw', f'{yaw0}',
                   '--frame-id', 'world', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # Optional cone, 1.5 m ahead of the robot and facing the same way.
    cone_nodes = []
    if cone:
        cone_nodes = [
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=['-file', os.path.join(share, 'models', 'cone.sdf'), '-name', 'cone',
                           '-x', f'{x0 + 1.5 * math.cos(yaw0):.4f}',
                           '-y', f'{y0 + 1.5 * math.sin(yaw0):.4f}',
                           '-z', '0.0', '-Y', f'{yaw0}'],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_cone',
                output='screen',
                parameters=[{'config_file': os.path.join(share, 'config', 'bridge_cone.yaml'),
                             'use_sim_time': True}],
            ),
        ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(share, 'config', 'a2.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Optional reference path for pure pursuit, drawn in RViz. Controllers
    # read the same waypoints from paths/<name>.csv.
    path_name = LaunchConfiguration('path').perform(context).strip().lower()
    if path_name not in ('', 'circle', 'square'):
        raise RuntimeError(f"path must be 'circle', 'square' or empty, got '{path_name}'")
    path_publisher = []
    if path_name:
        path_publisher = [Node(
            package=PKG,
            executable='path_publisher.py',
            output='screen',
            parameters=[{'path': path_name, 'use_sim_time': True}],
        )]

    summary = LogInfo(msg=(
        f'\n[A2] pair {pair}: physical r_left = {r_l:.6f} m, r_right = {r_r:.6f} m, '
        f'b = {b:.4f} m\n'
        f'[A2] diff-drive plugin told: wheel_radius = {r_mean:.6f} m, '
        f'wheel_separation = {b * scale:.4f} m (scale {scale:g})\n'
        f'[A2] LiDAR range noise: '
        f'{f"{lidar_noise:g} m std dev" if lidar_noise > 0 else "off"}\n'
        f'[A2] spawn: x = {x0:g} m, y = {y0:g} m, yaw = {yaw0:g} rad'
        f'{" (+ cone 1.5 m ahead)" if cone else ""}\n'))

    return [summary, *gazebo, robot_state_publisher, spawn, bridge,
            bridge_plugin_tf, bridge_gt_tf, world_to_odom, rviz, *path_publisher,
            *cone_nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pair', default_value='REF',
                              description='Your pair ID (P01..P12) or REF'),
        DeclareLaunchArgument('plugin_separation_scale', default_value='1.0',
                              description="Scale on the diff-drive plugin's wheel_separation"),
        DeclareLaunchArgument('lidar_noise', default_value='0.01',
                              description='Std dev (m) of LiDAR range noise; 0 = off'),
        DeclareLaunchArgument('plugin_tf', default_value='true',
                              description="Bridge the plugin's odom->base_link TF"),
        DeclareLaunchArgument('gt_tf', default_value='true',
                              description='Bridge the ground-truth world->base_link_gt TF'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('path', default_value='',
                              description='Reference path to show in RViz: circle, square or empty'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x in the world frame (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y in the world frame (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='Spawn heading in the world frame (rad)'),
        DeclareLaunchArgument('cone', default_value='false',
                              description='Add the movable cone 1.5 m ahead of the robot'),
        OpaqueFunction(function=launch_setup),
    ])
