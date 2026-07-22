"""Phase 1 multi-ASV: two independently controllable boats, no mission yet."""

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
        DeclareLaunchArgument('fast', default_value='false'),
        DeclareLaunchArgument('plot_trajectory', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_asvs': '2',
                'autonomy': 'false',
                'mission': 'false',
                'sensing': 'false',
                'mapping': 'false',
                'headless': LaunchConfiguration('headless'),
                'fast': LaunchConfiguration('fast'),
                'plot_trajectory': LaunchConfiguration('plot_trajectory'),
            }.items(),
        ),
    ])
