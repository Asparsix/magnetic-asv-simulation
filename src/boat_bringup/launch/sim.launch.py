"""Launch the standalone Gazebo boat and its ROS control path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('boat_description')
    bringup_share = get_package_share_directory('boat_bringup')
    world = os.path.join(description_share, 'worlds', 'water_world.sdf')
    models = os.path.join(description_share, 'models')
    bridge_config = os.path.join(bringup_share, 'config', 'bridge.yaml')
    navigation_config = os.path.join(bringup_share, 'config', 'navigation.yaml')
    sensing_config = os.path.join(bringup_share, 'config', 'sensing.yaml')
    mapping_config = os.path.join(bringup_share, 'config', 'mapping.yaml')
    mission_config = os.path.join(bringup_share, 'config', 'mission.yaml')

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models
    if existing_resource_path:
        resource_path += os.pathsep + existing_resource_path

    headless = LaunchConfiguration('headless')
    autonomy = LaunchConfiguration('autonomy')
    plot_trajectory = LaunchConfiguration('plot_trajectory')
    sensing = LaunchConfiguration('sensing')
    mapping = LaunchConfiguration('mapping')
    mission = LaunchConfiguration('mission')
    asv_ns = LaunchConfiguration('asv_ns')

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
            'asv_ns',
            default_value='asv1',
            description='ROS namespace for this ASV stack.',
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen',
            condition=UnlessCondition(headless),
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', world],
            output='screen',
            condition=IfCondition(headless),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='boat_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
        Node(
            package='boat_control',
            executable='thrust_mixer',
            name='thrust_mixer',
            namespace=asv_ns,
            parameters=[{
                'model_name': 'simple_boat',
                'cmd_vel_topic': 'cmd_vel',
                'thrust_scale': 50.0,
                'turn_gain': 1.0,
                'max_thrust': 100.0,
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
                {'use_builtin_path': False},
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
            parameters=[navigation_config],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", autonomy, "' == 'true' and '", mission, "' == 'false'",
            ])),
        ),
        Node(
            package='boat_navigation',
            executable='gazebo_pose2d',
            name='gazebo_pose2d',
            namespace=asv_ns,
            parameters=[{
                'model_name': 'simple_boat',
                'world_name': 'niot_world',
                'pose_topic': 'pose2d',
            }],
            output='screen',
        ),
        Node(
            package='boat_sensing',
            executable='mag_driver',
            name='mag_driver',
            namespace=asv_ns,
            parameters=[sensing_config],
            output='screen',
            condition=IfCondition(sensing),
        ),
        Node(
            package='boat_sensing',
            executable='mag_filter',
            name='mag_filter',
            namespace=asv_ns,
            parameters=[sensing_config],
            output='screen',
            condition=IfCondition(sensing),
        ),
        Node(
            package='boat_sensing',
            executable='calibration_node',
            name='calibration_node',
            namespace=asv_ns,
            parameters=[sensing_config],
            output='screen',
            condition=IfCondition(sensing),
        ),
        Node(
            package='boat_mapping',
            executable='bayes_fusion',
            name='bayes_fusion',
            parameters=[mapping_config],
            output='screen',
            condition=IfCondition(mapping),
        ),
        Node(
            package='boat_mission',
            executable='mission_manager',
            name='mission_manager',
            namespace=asv_ns,
            parameters=[mission_config],
            output='screen',
            condition=IfCondition(mission),
        ),
        Node(
            package='boat_mission',
            executable='verify_coordinator',
            name='verify_coordinator',
            parameters=[mission_config],
            output='screen',
            condition=IfCondition(mission),
        ),
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
                'show_true_target': True,
                'target_x': -50.0,
                'target_y': 70.0,
                'anomaly_vmax_nt': 55.0,
                'anomaly_hit_threshold_nt': 15.0,
            }],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", plot_trajectory, "' == 'true' and '",
                headless, "' == 'false'",
            ])),
        ),
    ])