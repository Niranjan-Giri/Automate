import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    params_file = os.path.join(
        get_package_share_directory('Navigation'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='Navigation',
            executable='occupancy_grid',
            name='occupancy_grid',
            parameters=[{params_file}, {'use_sim_time': True}]
        ),
        Node(
            package='Navigation',
            executable='local_costmap',
            name='local_costmap',
            parameters=[{params_file}, {'use_sim_time': True}]
        )
    ])