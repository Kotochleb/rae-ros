import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter

def launch_setup(context, *args, **kwargs):
    rae_description_pkg = get_package_share_directory('rae_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    rae_gazebo_pkg = get_package_share_directory('rae_gazebo')

    world_file = LaunchConfiguration('sdf_file').perform(context)
    print(f'world_file: {world_file}')

    spawn_robot = LaunchConfiguration('spawn_robot')
    namespace = LaunchConfiguration('namespace')
    enable_localization = LaunchConfiguration('enable_localization')

    return [
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                str(Path(rae_description_pkg).parent.resolve())
            ]
        ),

        # Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': world_file,
                'use_sim_time': 'True'
            }.items()
        ),

        # ros_gz_bridge with common params
        Node(
            package='ros_gz_bridge',
            namespace=LaunchConfiguration('namespace'),
            executable='parameter_bridge',
            parameters=[{
                'expand_gz_topic_names': True,
                'use_sim_time': True,
                'config_file': os.path.join(get_package_share_directory(
                    'rae_gazebo'),'config', 'gz_bridge_common.yaml')
            }],
            output='screen'
        ),

        # Spawn single rae
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rae_gazebo_pkg, 'launch', 'spawn_rae.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'enable_localization': enable_localization
            }.items(),
            condition=IfCondition(spawn_robot)
        ),

        # Set all nodes to use simulation time
        SetParameter(name='use_sim_time', value=True)
    ]



def generate_launch_description():
    rae_gazebo_pkg = get_package_share_directory('rae_gazebo')
    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument('enable_localization', default_value='true'),
        DeclareLaunchArgument('sdf_file', default_value=f'-r {os.path.join(rae_gazebo_pkg, "worlds", "world_demo.sdf")}'),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )