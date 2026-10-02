import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tb3_sim_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_sim_launch),
            launch_arguments={'slam': 'True', 'headless': 'False'}.items(),
        ),
    ])
