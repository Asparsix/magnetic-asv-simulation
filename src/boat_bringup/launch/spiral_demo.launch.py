"""Run an isolated transit-then-production-spiral tracking demonstration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('boat_bringup')
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')
    navigation_config = os.path.join(
        bringup_share, 'config', 'navigation.yaml'
    )

    return LaunchDescription([
        # Open water: island is at (40,30) r=12, target at (-50,70), shores at +/-155.
        DeclareLaunchArgument('center_x', default_value='-30.0'),
        DeclareLaunchArgument('center_y', default_value='-40.0'),
        DeclareLaunchArgument('spiral_max_radius_m', default_value='45.0'),
        DeclareLaunchArgument('spiral_ring_spacing_m', default_value='15.0'),
        DeclareLaunchArgument('spiral_step_spacing_m', default_value='10.0'),
        DeclareLaunchArgument('fast', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'headless': 'false',
                'plot_trajectory': 'true',
                'autonomy': 'false',
                'mission': 'false',
                'sensing': 'false',
                'mapping': 'false',
                'fast': LaunchConfiguration('fast'),
            }.items(),
        ),
        Node(
            package='boat_navigation',
            executable='los_path_follower',
            name='los_path_follower',
            namespace='asv1',
            parameters=[
                navigation_config,
                {
                    'use_builtin_path': False,
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('fast'), value_type=bool
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='boat_mission',
            executable='spiral_demo',
            name='spiral_demo',
            namespace='asv1',
            parameters=[{
                'center_x': ParameterValue(
                    LaunchConfiguration('center_x'), value_type=float
                ),
                'center_y': ParameterValue(
                    LaunchConfiguration('center_y'), value_type=float
                ),
                'spiral_max_radius_m': ParameterValue(
                    LaunchConfiguration('spiral_max_radius_m'),
                    value_type=float,
                ),
                'spiral_ring_spacing_m': ParameterValue(
                    LaunchConfiguration('spiral_ring_spacing_m'),
                    value_type=float,
                ),
                'spiral_step_spacing_m': ParameterValue(
                    LaunchConfiguration('spiral_step_spacing_m'),
                    value_type=float,
                ),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('fast'), value_type=bool
                ),
            }],
            output='screen',
        ),
    ])
