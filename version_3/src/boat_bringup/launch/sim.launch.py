"""Launch the Gazebo lake world and one or two independent ASV stacks."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


ASV_SPECS = (
    {'ns': 'asv1', 'model': 'simple_boat'},
    {'ns': 'asv2', 'model': 'simple_boat_2'},
    {'ns': 'asv3', 'model': 'simple_boat_3'},
)


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _load_trial_params(context):
    """Optional Monte Carlo trial overrides (YAML dict)."""
    path = LaunchConfiguration('trial_params_file').perform(context).strip()
    if not path:
        return {}
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f'trial_params_file must be a YAML mapping: {path}')
    return data


def _fast_world_sdf(sdf, factor, step_size):
    """Create a bounded fast-mode world with an explicit integration step."""
    factor = float(factor)
    if factor <= 1.0 or factor > 10.0:
        raise ValueError('fast_factor must be > 1.0 and <= 10.0')
    step_size = float(step_size)
    if step_size < 0.001 or step_size > 0.004:
        raise ValueError('fast_step_size must be between 0.001 and 0.004')
    rtf_old = '<real_time_factor>1.0</real_time_factor>'
    step_old = '<max_step_size>0.001</max_step_size>'
    if rtf_old not in sdf or step_old not in sdf:
        raise RuntimeError('Expected baseline physics settings not found')
    sdf = sdf.replace(
        rtf_old,
        f'<real_time_factor>{factor:g}</real_time_factor>',
        1,
    )
    return sdf.replace(
        step_old,
        f'<max_step_size>{step_size:g}</max_step_size>',
        1,
    )


def _gazebo_process(context, world):
    """Launch Gazebo, optionally with only real_time_factor changed."""
    headless = _as_bool(LaunchConfiguration('headless').perform(context))
    fast = _as_bool(LaunchConfiguration('fast').perform(context))
    selected_world = world

    if fast:
        factor = float(LaunchConfiguration('fast_factor').perform(context))
        step_size = float(
            LaunchConfiguration('fast_step_size').perform(context)
        )
        with open(world, encoding='utf-8') as source:
            sdf = source.read()
        sdf = _fast_world_sdf(sdf, factor, step_size)
        fd, selected_world = tempfile.mkstemp(
            prefix='boat_fast_', suffix='.sdf'
        )
        with os.fdopen(fd, 'w', encoding='utf-8') as output:
            output.write(sdf)

    cmd = ['gz', 'sim']
    if headless:
        cmd.append('-s')
    cmd.extend(['-r', selected_world])
    return [ExecuteProcess(cmd=cmd, output='screen')]


def _asv_stack(
    *,
    asv_ns,
    model_name,
    use_sim_time,
    autonomy,
    mission,
    sensing,
    plot_trajectory,
    headless,
    navigation_config,
    sensing_config,
    mission_config,
    lawnmower_asv_index,
    lawnmower_num_asvs,
    self_verify,
    enable_mission,
    enable_verify_coordinator,
    enable_plotter,
    trial_params=None,
):
    """Build the per-ASV control / sensing / optional mission nodes."""
    trial_params = trial_params or {}
    mag_overrides = dict(trial_params.get('mag_driver', {}))
    mission_overrides = dict(trial_params.get('mission_manager', {}))
    nav_overrides = dict(trial_params.get('los_path_follower', {}))
    nodes = [
        Node(
            package='boat_control',
            executable='thrust_mixer',
            name='thrust_mixer',
            namespace=asv_ns,
            parameters=[use_sim_time, {
                'model_name': model_name,
                'cmd_vel_topic': 'cmd_vel',
                'thrust_scale': 50.0,
                'turn_gain': 1.0,
                'max_thrust': 100.0,
            }],
            output='screen',
        ),
        Node(
            package='boat_navigation',
            executable='gazebo_pose2d',
            name='gazebo_pose2d',
            namespace=asv_ns,
            parameters=[use_sim_time, {
                'model_name': model_name,
                'world_name': 'niot_world',
                'pose_topic': 'pose2d',
            }],
            output='screen',
        ),
        Node(
            package='boat_navigation',
            executable='los_path_follower',
            name='los_path_follower',
            namespace=asv_ns,
            parameters=[
                navigation_config,
                use_sim_time,
                {'use_builtin_path': False},
                nav_overrides,
            ],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", autonomy, "' == 'true' and '", mission, "' == 'true'",
            ])),
        ),
        Node(
            package='boat_navigation',
            executable='los_path_follower',
            name='los_path_follower',
            namespace=asv_ns,
            parameters=[navigation_config, use_sim_time, nav_overrides],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", autonomy, "' == 'true' and '", mission, "' == 'false'",
            ])),
        ),
        Node(
            package='boat_sensing',
            executable='mag_driver',
            name='mag_driver',
            namespace=asv_ns,
            parameters=[
                sensing_config,
                use_sim_time,
                {'frame_id': f'{asv_ns}/mag_link'},
                mag_overrides,
            ],
            output='screen',
            condition=IfCondition(sensing),
        ),
        Node(
            package='boat_sensing',
            executable='mag_filter',
            name='mag_filter',
            namespace=asv_ns,
            parameters=[sensing_config, use_sim_time],
            output='screen',
            condition=IfCondition(sensing),
        ),
        Node(
            package='boat_sensing',
            executable='calibration_node',
            name='calibration_node',
            namespace=asv_ns,
            parameters=[sensing_config, use_sim_time],
            output='screen',
            condition=IfCondition(sensing),
        ),
    ]

    if enable_mission:
        nodes.append(
            Node(
                package='boat_mission',
                executable='mission_manager',
                name='mission_manager',
                namespace=asv_ns,
                parameters=[
                    mission_config,
                    use_sim_time,
                    {
                        'lawnmower_asv_index': lawnmower_asv_index,
                        'lawnmower_num_asvs': lawnmower_num_asvs,
                        'self_verify': self_verify,
                        **mission_overrides,
                    },
                ],
                output='screen',
                condition=IfCondition(mission),
            )
        )

    if enable_verify_coordinator:
        nodes.append(
            Node(
                package='boat_mission',
                executable='verify_coordinator',
                name='verify_coordinator',
                parameters=[mission_config, use_sim_time],
                output='screen',
                condition=IfCondition(mission),
            )
        )

    if enable_plotter:
        nodes.append(
            Node(
                package='boat_navigation',
                executable='trajectory_plotter',
                name='trajectory_plotter',
                namespace=asv_ns,
                parameters=[{
                    'pose_topic': 'pose2d',
                    'trajectory_topic': 'trajectory',
                    'active_plan_topic': 'plan/active',
                    'anomaly_topic': 'mag/anomaly',
                    'peak_topic': '/swarm/belief/peak',
                    'peak_probability_topic': '/swarm/belief/peak_probability',
                    'centroid_topic': '/swarm/belief/centroid',
                    'centroid_spread_topic': '/swarm/belief/centroid_spread',
                    'fix_topic': '/swarm/belief/fix',
                    'fix_rms_topic': '/swarm/belief/fix_rms',
                    'show_true_target': True,
                    'target_x': 0.0,
                    'target_y': -85.0,
                    'lake_half_size_m': 150.0,
                    'anomaly_vmax_nt': 55.0,
                    'anomaly_hit_threshold_nt': 15.0,
                }],
                output='screen',
                condition=IfCondition(PythonExpression([
                    "'", plot_trajectory, "' == 'true' and '",
                    headless, "' == 'false'",
                ])),
            )
        )
    return nodes


def _multi_asv_nodes(context, *args):
    """Instantiate one to three ASV stacks from num_asvs."""
    bringup_share = get_package_share_directory('boat_bringup')
    navigation_config = os.path.join(bringup_share, 'config', 'navigation.yaml')
    sensing_config = os.path.join(bringup_share, 'config', 'sensing.yaml')
    mapping_config = os.path.join(bringup_share, 'config', 'mapping.yaml')
    mission_config = os.path.join(bringup_share, 'config', 'mission.yaml')

    num_asvs = int(LaunchConfiguration('num_asvs').perform(context))
    if num_asvs not in (1, 2, 3):
        raise ValueError('num_asvs must be 1, 2, or 3')

    trial_params = _load_trial_params(context)
    mapping_overrides = dict(trial_params.get('bayes_fusion', {}))
    plotter_overrides = dict(trial_params.get('plotter', {}))

    fast = LaunchConfiguration('fast')
    use_sim_time = {
        'use_sim_time': ParameterValue(fast, value_type=bool),
    }
    autonomy = LaunchConfiguration('autonomy')
    mission = LaunchConfiguration('mission')
    sensing = LaunchConfiguration('sensing')
    mapping = LaunchConfiguration('mapping')
    plot_trajectory = LaunchConfiguration('plot_trajectory')
    headless = LaunchConfiguration('headless')

    nodes = []
    for index, spec in enumerate(ASV_SPECS[:num_asvs]):
        nodes.extend(
            _asv_stack(
                asv_ns=spec['ns'],
                model_name=spec['model'],
                use_sim_time=use_sim_time,
                autonomy=autonomy,
                mission=mission,
                sensing=sensing,
                plot_trajectory=plot_trajectory,
                headless=headless,
                navigation_config=navigation_config,
                sensing_config=sensing_config,
                mission_config=mission_config,
                lawnmower_asv_index=index,
                lawnmower_num_asvs=num_asvs,
                # Multi-ASV: discoverer HOLDs; peer verifies (Phase 4).
                self_verify=(num_asvs == 1),
                enable_mission=True,
                enable_verify_coordinator=(index == 0),
                # Single boat keeps its own namespaced plotter; multi-ASV uses
                # one shared plotter (added below) that draws every boat.
                enable_plotter=(index == 0 and num_asvs == 1),
                trial_params=trial_params,
            )
        )

    if num_asvs > 1:
        namespaces = [spec['ns'] for spec in ASV_SPECS[:num_asvs]]
        plotter_params = {
            'asv_namespaces': namespaces,
            'pose_topic': 'pose2d',
            'trajectory_topic': 'trajectory',
            'active_plan_topic': 'plan/active',
            'anomaly_topic': 'mag/anomaly',
            'peak_topic': '/swarm/belief/peak',
            'peak_probability_topic': '/swarm/belief/peak_probability',
            'centroid_topic': '/swarm/belief/centroid',
            'centroid_spread_topic': '/swarm/belief/centroid_spread',
            'fix_topic': '/swarm/belief/fix',
            'fix_rms_topic': '/swarm/belief/fix_rms',
            'show_true_target': True,
            'target_x': 0.0,
            'target_y': -85.0,
            'lake_half_size_m': 150.0,
            'anomaly_vmax_nt': 55.0,
            'anomaly_hit_threshold_nt': 15.0,
            **plotter_overrides,
        }
        nodes.append(
            Node(
                package='boat_navigation',
                executable='trajectory_plotter',
                name='trajectory_plotter',
                parameters=[plotter_params],
                output='screen',
                condition=IfCondition(PythonExpression([
                    "'", plot_trajectory, "' == 'true' and '",
                    headless, "' == 'false'",
                ])),
            )
        )

    nodes.append(
        Node(
            package='boat_mapping',
            executable='bayes_fusion',
            name='bayes_fusion',
            parameters=[mapping_config, use_sim_time, mapping_overrides],
            output='screen',
            condition=IfCondition(mapping),
        )
    )
    return nodes


def generate_launch_description():
    description_share = get_package_share_directory('boat_description')
    bringup_share = get_package_share_directory('boat_bringup')
    world = os.path.join(description_share, 'worlds', 'water_world.sdf')
    models = os.path.join(description_share, 'models')
    bridge_config = os.path.join(bringup_share, 'config', 'bridge.yaml')
    clock_bridge_config = os.path.join(
        bringup_share, 'config', 'clock_bridge.yaml'
    )

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models
    if existing_resource_path:
        resource_path += os.pathsep + existing_resource_path

    fast = LaunchConfiguration('fast')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the Gazebo server without the GUI.',
        ),
        DeclareLaunchArgument(
            'autonomy',
            default_value='true',
            description='Enable LOS path following on cmd_vel.',
        ),
        DeclareLaunchArgument(
            'plot_trajectory',
            default_value='true',
            description='Open the live trajectory and heading plot.',
        ),
        DeclareLaunchArgument(
            'sensing',
            default_value='true',
            description='Enable mag_driver, mag_filter, and calibration nodes.',
        ),
        DeclareLaunchArgument(
            'mapping',
            default_value='true',
            description='Enable shared Bayesian belief fusion.',
        ),
        DeclareLaunchArgument(
            'mission',
            default_value='true',
            description='Enable mission manager (lawnmower + mode switch).',
        ),
        DeclareLaunchArgument(
            'num_asvs',
            default_value='1',
            description='Number of independent ASV stacks to launch (1, 2, or 3).',
        ),
        DeclareLaunchArgument(
            'fast',
            default_value='false',
            description=(
                'Run physics faster than wall time and clock ROS nodes from '
                'Gazebo simulation time.'
            ),
        ),
        DeclareLaunchArgument(
            'fast_factor',
            default_value='2.0',
            description='Requested Gazebo real-time factor in fast mode (1-10).',
        ),
        DeclareLaunchArgument(
            'fast_step_size',
            default_value='0.002',
            description=(
                'Physics integration step in fast mode (0.001-0.004 seconds).'
            ),
        ),
        DeclareLaunchArgument(
            'trial_params_file',
            default_value='',
            description=(
                'Optional YAML file with per-trial overrides for Monte Carlo '
                '(mag_driver, mission_manager, bayes_fusion, plotter).'
            ),
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        OpaqueFunction(function=_gazebo_process, args=[world]),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='boat_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            parameters=[{'config_file': clock_bridge_config}],
            output='screen',
            condition=IfCondition(fast),
        ),
        OpaqueFunction(function=_multi_asv_nodes),
    ])
