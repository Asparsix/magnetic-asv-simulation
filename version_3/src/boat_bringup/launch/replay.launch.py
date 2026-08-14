"""Replay a recorded bag into sensing/mapping/mission without Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _replay_cmd(bag_path, rate, mode):
    """Build the ros2 bag play command for a given replay mode."""
    mode = str(mode).strip().lower()
    if not bag_path:
        raise RuntimeError('Pass bag:=/path/to/recorded_bag')
    if not os.path.exists(bag_path):
        raise RuntimeError(f'Bag path does not exist: {bag_path}')

    # from_pose: only drive sensing from recorded motion.
    # from_anomaly: feed Bayes/mission from recorded cleaned anomalies.
    if mode == 'from_pose':
        topics = ['/asv1/pose2d', '/asv1/cmd_vel']
    elif mode == 'from_anomaly':
        topics = ['/asv1/mag/anomaly']
    else:
        raise RuntimeError(
            "mode must be 'from_pose' or 'from_anomaly', got: " + mode
        )

    return [
        'ros2', 'bag', 'play', bag_path,
        '--clock',
        '--rate', str(rate),
        '--topics',
    ] + topics


def _bag_player(context):
    bag_path = LaunchConfiguration('bag').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    cmd = _replay_cmd(bag_path, rate, mode)
    return [
        LogInfo(msg=[
            'Replaying ', bag_path, ' mode=', mode,
            ' rate=', rate, 'x (no Gazebo)',
        ]),
        ExecuteProcess(cmd=cmd, output='screen'),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('boat_bringup')
    sensing_config = os.path.join(bringup_share, 'config', 'sensing.yaml')
    mapping_config = os.path.join(bringup_share, 'config', 'mapping.yaml')
    mission_config = os.path.join(bringup_share, 'config', 'mission.yaml')

    mode = LaunchConfiguration('mode')
    plot_trajectory = LaunchConfiguration('plot_trajectory')
    asv_ns = LaunchConfiguration('asv_ns')
    use_sim_time = {'use_sim_time': True}

    from_pose = IfCondition(
        PythonExpression(["'", mode, "' == 'from_pose'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag',
            default_value='',
            description='Absolute path to a recorded rosbag2 directory.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='from_pose',
            description=(
                "from_pose: regenerate mag/filter/calibration/Bayes from "
                "recorded pose. from_anomaly: replay MagAnomaly into "
                "Bayes/mission only."
            ),
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='5.0',
            description='Playback speed multiplier (5.0 = five times faster).',
        ),
        DeclareLaunchArgument(
            'plot_trajectory',
            default_value='true',
            description='Open the trajectory/anomaly plotter.',
        ),
        DeclareLaunchArgument(
            'asv_ns',
            default_value='asv1',
            description='ASV namespace used when the bag was recorded.',
        ),
        OpaqueFunction(function=_bag_player),
        # --- from_pose: rebuild the magnetic pipeline from motion ---
        Node(
            package='boat_sensing',
            executable='mag_driver',
            name='mag_driver',
            namespace=asv_ns,
            parameters=[sensing_config, use_sim_time],
            output='screen',
            condition=from_pose,
        ),
        Node(
            package='boat_sensing',
            executable='mag_filter',
            name='mag_filter',
            namespace=asv_ns,
            parameters=[sensing_config, use_sim_time],
            output='screen',
            condition=from_pose,
        ),
        Node(
            package='boat_sensing',
            executable='calibration_node',
            name='calibration_node',
            namespace=asv_ns,
            parameters=[sensing_config, use_sim_time],
            output='screen',
            condition=from_pose,
        ),
        # --- both modes: belief + mission ---
        Node(
            package='boat_mapping',
            executable='bayes_fusion',
            name='bayes_fusion',
            parameters=[mapping_config, use_sim_time],
            output='screen',
        ),
        Node(
            package='boat_mission',
            executable='mission_manager',
            name='mission_manager',
            namespace=asv_ns,
            parameters=[mission_config, use_sim_time],
            output='screen',
        ),
        Node(
            package='boat_mission',
            executable='verify_coordinator',
            name='verify_coordinator',
            parameters=[mission_config, use_sim_time],
            output='screen',
        ),
        Node(
            package='boat_navigation',
            executable='trajectory_plotter',
            name='trajectory_plotter',
            namespace=asv_ns,
            parameters=[use_sim_time, {
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
                'target_x': 85.0,
                'target_y': 40.0,
                'anomaly_vmax_nt': 55.0,
                'anomaly_hit_threshold_nt': 15.0,
            }],
            output='screen',
            condition=IfCondition(plot_trajectory),
        ),
    ])
