import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('rosetta_bringup')
    launch_dir = os.path.join(bringup_share, 'launch')

    # Load defaults from each params file
    with open(os.path.join(bringup_share, 'params', 'gazebo_world.yaml')) as f:
        gz_defaults = yaml.safe_load(f)['gazebo_world']
    with open(os.path.join(bringup_share, 'params', 'teleop.yaml')) as f:
        teleop_defaults = yaml.safe_load(f)['teleop']
    with open(os.path.join(bringup_share, 'params', 'zenoh.yaml')) as f:
        zenoh_defaults = yaml.safe_load(f)['zenoh']

    # Top-level launch args (forwarded to child launches)
    turtlebot3_model = LaunchConfiguration('turtlebot3_model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist')
    zenoh_enabled = LaunchConfiguration('zenoh_enabled')

    return LaunchDescription([
        # -- Declare top-level arguments --
        DeclareLaunchArgument(
            'turtlebot3_model',
            default_value=str(gz_defaults['turtlebot3_model']),
            description='TurtleBot3 model type (burger, waffle, waffle_pi)',
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value=str(gz_defaults['x_pose']),
            description='X spawn position',
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value=str(gz_defaults['y_pose']),
            description='Y spawn position',
        ),
        DeclareLaunchArgument(
            'joy_config',
            default_value=str(teleop_defaults['joy_config']),
            description='Joystick configuration (ps3, xbox, etc.)',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value=str(teleop_defaults['joy_dev']),
            description='Joystick device ID',
        ),
        DeclareLaunchArgument(
            'publish_stamped_twist',
            default_value=str(teleop_defaults['publish_stamped_twist']),
            description='Publish stamped twist messages',
        ),
        DeclareLaunchArgument(
            'zenoh_enabled',
            default_value=str(zenoh_defaults['enabled']).lower(),
            description='Whether to start the Zenoh router daemon',
        ),

        # -- 1. Zenoh daemon --
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'zenoh_launch.py')
            ),
            launch_arguments={
                'zenoh_enabled': zenoh_enabled,
            }.items(),
        ),

        # -- 2. Gazebo world + robot spawn --
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo_world_launch.py')
            ),
            launch_arguments={
                'turtlebot3_model': turtlebot3_model,
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items(),
        ),

        # -- 3. Target spawner --
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'target_spawner_launch.py')
            ),
        ),

        # -- 4. Teleop --
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'teleop_launch.py')
            ),
            launch_arguments={
                'joy_config': joy_config,
                'joy_dev': joy_dev,
                'publish_stamped_twist': publish_stamped_twist,
            }.items(),
        ),
    ])
