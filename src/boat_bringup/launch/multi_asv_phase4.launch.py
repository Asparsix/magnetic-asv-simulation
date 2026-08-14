"""Phase 4 multi-ASV on the 1 km lake: 3 boats, cooperative discover/verify.

Shoreline starts at SW / SE / NE corners with Voronoi regional coverage.
Discoverer HOLDs after spiral; coordinator assigns another ASV to verify
from the opposite approach side.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('boat_bringup')
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('fast', default_value='true'),
        DeclareLaunchArgument('plot_trajectory', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_asvs': '3',
                'autonomy': 'true',
                'mission': 'true',
                'sensing': 'true',
                'mapping': 'true',
                'headless': LaunchConfiguration('headless'),
                'fast': LaunchConfiguration('fast'),
                'plot_trajectory': LaunchConfiguration('plot_trajectory'),
                'fast_factor': '5.0',
                'fast_step_size': '0.003',
            }.items(),
        ),
    ])
