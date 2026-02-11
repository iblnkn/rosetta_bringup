"""
Launch the Episode Recorder node for the TurtleBot3 simulation.

Wraps the canonical rosetta/episode_recorder_launch.py with sim-specific
defaults: turtlebot3 contract, use_sim_time, and log level.

Usage:
    # Launch with sim defaults
    ros2 launch rosetta_bringup turtlebot_recorder_launch.py

    # Override any parameter
    ros2 launch rosetta_bringup turtlebot_recorder_launch.py \
        bag_base_dir:=/tmp/my_bags log_level:=debug
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('rosetta_bringup')
    rosetta_share = get_package_share_directory('rosetta')

    with open(os.path.join(bringup_share, 'params', 'turtlebot_recorder.yaml')) as f:
        overrides = yaml.safe_load(f)['turtlebot_recorder']

    default_contract = os.path.join(
        rosetta_share, 'contracts', overrides['contract'],
    )

    contract_path = LaunchConfiguration('contract_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    configure = LaunchConfiguration('configure')
    activate = LaunchConfiguration('activate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'contract_path',
            default_value=default_contract,
            description='Path to the robot contract YAML',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(overrides.get('use_sim_time', True)).lower(),
            description='Use simulated clock from Gazebo',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value=str(overrides.get('log_level', 'info')),
            description='Logging level (debug, info, warn, error)',
        ),
        DeclareLaunchArgument(
            'configure',
            default_value=str(overrides.get('configure', True)).lower(),
            description='Auto-configure the lifecycle node on startup',
        ),
        DeclareLaunchArgument(
            'activate',
            default_value=str(overrides.get('activate', True)).lower(),
            description='Auto-activate the lifecycle node on startup',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosetta_share, 'launch', 'episode_recorder_launch.py')
            ),
            launch_arguments={
                'contract_path': contract_path,
                'use_sim_time': use_sim_time,
                'log_level': log_level,
                'configure': configure,
                'activate': activate,
            }.items(),
        ),
    ])
