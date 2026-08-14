"""Headless fast 3-ASV launch with optional Monte Carlo trial parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _load_trial_params(context):
    path = LaunchConfiguration('trial_params_file').perform(context).strip()
    if not path:
        return {}
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else {}


def _monte_carlo_drift_nodes(context, *args):
    """MC-only mag baseline drift nodes (not used in normal sim)."""
    trial = _load_trial_params(context)
    drift = trial.get('mag_baseline_drift')
    if not drift:
        return []

    fast = LaunchConfiguration('fast')
    use_sim_time = {'use_sim_time': ParameterValue(fast, value_type=bool)}
    vert_frac = float(drift.get('drift_vertical_fraction', 0.15))
    nodes = []
    for ns in ('asv1', 'asv2', 'asv3'):
        nodes.append(
            Node(
                package='boat_sensing',
                executable='monte_carlo_mag_drift',
                name='monte_carlo_mag_drift',
                namespace=ns,
                parameters=[
                    use_sim_time,
                    {
                        'input_topic': 'mag/plant',
                        'output_topic': 'mag/raw',
                        'drift_nt_per_min': float(
                            drift.get('drift_nt_per_min', 0.0)
                        ),
                        'drift_azimuth_deg': float(
                            drift.get('drift_azimuth_deg', 0.0)
                        ),
                        'drift_vertical_fraction': vert_frac,
                    },
                ],
                output='screen',
            )
        )
    return nodes


def generate_launch_description():
    bringup_share = get_package_share_directory('boat_bringup')
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'trial_params_file',
            default_value='',
            description='YAML overrides written by monte_carlo_run.py.',
        ),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('fast', default_value='true'),
        DeclareLaunchArgument('plot_trajectory', default_value='false'),
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
                'trial_params_file': LaunchConfiguration('trial_params_file'),
            }.items(),
        ),
        OpaqueFunction(function=_monte_carlo_drift_nodes),
    ])
