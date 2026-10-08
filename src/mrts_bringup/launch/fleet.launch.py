from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrts_fleet_manager',
            executable='fleet_manager_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
