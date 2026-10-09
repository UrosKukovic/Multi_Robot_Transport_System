import os
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml

# SLAM started at Gazebo world (-2.0, -0.5), so that world point is map (0, 0).
# world = map + MAP_ORIGIN_IN_WORLD
MAP_ORIGIN_IN_WORLD = (-2.0, -0.5)


def load_robots(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)['robots']


def robot_group(robot, nav2_dir, map_yaml, params_yaml, use_rviz):
    wx = robot['x'] + MAP_ORIGIN_IN_WORLD[0]
    wy = robot['y'] + MAP_ORIGIN_IN_WORLD[1]

    # Same shared params file, but AMCL's initial pose is overridden per robot.
    robot_params = RewrittenYaml(
        source_file=params_yaml,
        param_rewrites={
            'amcl.ros__parameters.initial_pose.x': str(robot['x']),
            'amcl.ros__parameters.initial_pose.y': str(robot['y']),
            'amcl.ros__parameters.initial_pose.yaw': str(robot['yaw']),
        },
        convert_types=True,
    )

    return GroupAction([
        # Every robot's gz bridge (spawn_tb3.launch.py) also bridges the absolute /clock.
        # Two publishers of one clock interleave out of order -> sim time steps backwards
        # -> TF buffers are cleared -> Nav2 container dies on a TF LookupException.
        # Node-scoped remap: only this robot's bridge sends its clock to an unused topic;
        # the single clock_bridge below owns /clock.
        SetRemap(src='ros_gz_bridge:/clock', dst='clock_unused'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'rviz_launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={'namespace': robot['name']}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'tb3_simulation_launch.py')),
            launch_arguments={
                'namespace': robot['name'],
                'robot_name': robot['name'],
                'map': map_yaml,
                'params_file': robot_params,
                'use_sim_time': 'True',
                'autostart': 'True',
                'use_simulator': 'False',  # Gazebo is started once, below
                'use_rviz': 'False',       # RViz is started per robot, above
                'x_pose': str(wx),
                'y_pose': str(wy),
                'yaw': str(robot['yaw']),
            }.items(),
        ),
    ])


def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    mrts_dir = get_package_share_directory('mrts_bringup')

    map_yaml = os.path.join(mrts_dir, 'maps', 'warehouse.yaml')
    params_yaml = os.path.join(mrts_dir, 'config', 'nav2_params.yaml')
    robots_yaml = os.path.join(mrts_dir, 'config', 'robots.yaml')
    world = os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    use_rviz = LaunchConfiguration('use_rviz')

    data = load_robots(robots_yaml)

    # One Gazebo server for all robots (same as cloned_multi_tb3_simulation_launch.py).
    # mkstemp creates the file atomically (mktemp only picks a name -> race); xacro overwrites it.
    fd, world_sdf = tempfile.mkstemp(prefix='mrts_', suffix='.sdf')
    os.close(fd)

    world_xacro = ExecuteProcess(cmd=['xacro', '-o', world_sdf, 'headless:=False', world])

    ld = LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='True'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(Path(sim_dir).parent.resolve())),
        world_xacro,
        # Start Gazebo only after xacro has written the world file.
        RegisterEventHandler(OnProcessExit(
            target_action=world_xacro,
            on_exit=[ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world_sdf], output='screen')])),
        # The one and only /clock publisher (see SetRemap in robot_group).
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))])),
    ])

    for robot in data:
        ld.add_action(robot_group(robot, nav2_dir, map_yaml, params_yaml, use_rviz))

    return ld
