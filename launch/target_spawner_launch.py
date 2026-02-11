import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rosetta_bringup')
    params_file = os.path.join(bringup_share, 'params', 'target_spawner.yaml')

    return LaunchDescription([
        Node(
            package='turtlebot3_gazebo',
            executable='target_spawner.py',
            name='target_spawner',
            parameters=[params_file],
            output='screen',
        ),
    ])
