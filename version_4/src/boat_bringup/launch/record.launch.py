"""Record a Gazebo mission bag for later offline logic replay."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def _record_topics():
    bringup_share = get_package_share_directory('boat_bringup')
    config_path = os.path.join(bringup_share, 'config', 'record_topics.yaml')
    with open(config_path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    topics = list(data.get('topics', []))
    if not topics:
        raise RuntimeError(f'No topics listed in {config_path}')
    return topics


def _bag_recorder(context):
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context)
    if not bag_name:
        bag_name = datetime.now().strftime('mission_%Y%m%d_%H%M%S')
    os.makedirs(bag_dir, exist_ok=True)
    output = os.path.join(bag_dir, bag_name)
    topics = _record_topics()
    cmd = [
        'ros2', 'bag', 'record',
        '-o', output,
        '--compression-mode', 'file',
        '--compression-format', 'zstd',
    ] + topics
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    bringup_share = get_package_share_directory('boat_bringup')
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')
    default_bag_dir = os.path.join(
        os.path.expanduser('~'), 'simulation_ws', 'bags'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_dir',
            default_value=default_bag_dir,
            description='Directory where recorded bags are written.',
        ),
        DeclareLaunchArgument(
            'bag_name',
            default_value='',
            description=(
                'Bag folder name under bag_dir. Empty = timestamped name.'
            ),
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Forwarded to sim.launch.py.',
        ),
        DeclareLaunchArgument(
            'fast',
            default_value='false',
            description='Forwarded to sim.launch.py.',
        ),
        DeclareLaunchArgument(
            'plot_trajectory',
            default_value='false',
            description='Forwarded to sim.launch.py.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'fast': LaunchConfiguration('fast'),
                'plot_trajectory': LaunchConfiguration('plot_trajectory'),
                'mission': 'true',
                'sensing': 'true',
                'mapping': 'true',
                'autonomy': 'true',
            }.items(),
        ),
        OpaqueFunction(function=_bag_recorder),
    ])
