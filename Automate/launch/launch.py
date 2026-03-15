import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    params_file = os.path.join(
        get_package_share_directory('Automate'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='Automate',
            executable='static_tf',
            name='static_tf',
            parameters=[params_file]
        ),
        Node(
            package='Automate',
            executable='odometry',
            name='odometry',
            parameters=[params_file]
        ),
        Node(
            package='Automate',
            executable='map_tf',
            name='map_tf',
            parameters=[params_file]
        )
    ])