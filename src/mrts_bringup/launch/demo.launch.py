import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    fleet_launch = os.path.join(
        get_package_share_directory('mrts_bringup'), 'launch', 'fleet.launch.py'
    )

    multi_launch = os.path.join(
        get_package_share_directory('mrts_bringup'), 'launch', 'sim_multi.launch.py'
    )

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='True'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_launch),
            launch_arguments={'use_rviz': use_rviz}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fleet_launch),
        ),
    ])
