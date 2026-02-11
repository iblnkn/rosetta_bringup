import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('rosetta_bringup')
    params_file = os.path.join(bringup_share, 'params', 'gazebo_world.yaml')

    with open(params_file) as f:
        defaults = yaml.safe_load(f)['gazebo_world']

    turtlebot3_model = LaunchConfiguration('turtlebot3_model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'turtlebot3_model',
            default_value=str(defaults['turtlebot3_model']),
            description='TurtleBot3 model type (burger, waffle, waffle_pi)',
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value=str(defaults['x_pose']),
            description='X spawn position',
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value=str(defaults['y_pose']),
            description='Y spawn position',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(defaults['use_sim_time']).lower(),
            description='Use simulation clock',
        ),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items(),
        ),
    ])
