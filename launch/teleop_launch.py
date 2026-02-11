import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('rosetta_bringup')
    params_file = os.path.join(bringup_share, 'params', 'teleop.yaml')

    with open(params_file) as f:
        defaults = yaml.safe_load(f)['teleop']

    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist')

    teleop_share = get_package_share_directory('teleop_twist_joy')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_config',
            default_value=str(defaults['joy_config']),
            description='Joystick configuration (ps3, xbox, etc.)',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value=str(defaults['joy_dev']),
            description='Joystick device ID',
        ),
        DeclareLaunchArgument(
            'publish_stamped_twist',
            default_value=str(defaults['publish_stamped_twist']),
            description='Publish stamped twist messages',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_share, 'launch', 'teleop-launch.py')
            ),
            launch_arguments={
                'joy_config': joy_config,
                'joy_dev': joy_dev,
                'publish_stamped_twist': publish_stamped_twist,
            }.items(),
        ),
    ])
