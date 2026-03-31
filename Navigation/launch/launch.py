import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    params_file = os.path.join(
        get_package_share_directory('Navigation'),
        'config',
        'params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_waypoints = LaunchConfiguration('use_waypoints')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.'
        ),
        DeclareLaunchArgument(
            'use_waypoints',
            default_value='true',
            description='Enable waypoint planning from YAML file.'
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=os.path.join(
                get_package_share_directory('Navigation'),
                'config',
                'waypoints.yaml'
            ),
            description='Path to waypoint YAML file.'
        ),
        Node(
            package='Navigation',
            executable='occupancy_grid',
            name='occupancy_grid',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='Navigation',
            executable='local_costmap',
            name='local_costmap',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='Navigation',
            executable='global_planner',
            name='global_planner',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'use_waypoints': use_waypoints,
                    'waypoints_file': waypoints_file,
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        )
    ])