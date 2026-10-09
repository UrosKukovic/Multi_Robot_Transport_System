import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mrts_dir = get_package_share_directory('mrts_bringup')
    robots_yaml = os.path.join(mrts_dir, 'config', 'robots.yaml')
    with open(robots_yaml, 'r') as f:
        robots = yaml.safe_load(f)['robots']

    ids = [r['name'] for r in robots]

    return LaunchDescription([
        Node(
            package='mrts_fleet_manager',
            executable='fleet_manager_node',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_ids': ids}]
        ),
    ])
