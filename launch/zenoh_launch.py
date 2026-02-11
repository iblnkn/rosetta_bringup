import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('rosetta_bringup')
    params_file = os.path.join(bringup_share, 'params', 'zenoh.yaml')

    with open(params_file) as f:
        defaults = yaml.safe_load(f)['zenoh']

    enabled = LaunchConfiguration('zenoh_enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'zenoh_enabled',
            default_value=str(defaults['enabled']).lower(),
            description='Whether to start the Zenoh router daemon',
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
            output='screen',
            condition=IfCondition(enabled),
        ),
    ])
