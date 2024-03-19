import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter

from nav2_common.launch import ReplaceString

def launch_setup(context, *args, **kwargs):
    rae_description_pkg = get_package_share_directory('rae_description')

    namespace = LaunchConfiguration('namespace')
    enable_localization = LaunchConfiguration('enable_localization')

    ns_srt = namespace.perform(context)
    ns_srt = f'/{ns_srt}' if ns_srt else ''
    rae_name = ns_srt + '/rae' if ns_srt else 'rae'

    return [
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                str(Path(rae_description_pkg).parent.resolve())
            ]
        ),

        # RSP Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(rae_description_pkg, 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'True',
                'namespace': namespace
            }.items()
        ),

        # ros_gz_bridge
        Node(
            package='ros_gz_bridge',
            namespace=LaunchConfiguration('namespace'),
            executable='parameter_bridge',
            parameters=[{
                'expand_gz_topic_names': True,
                'use_sim_time': True,
                'config_file': os.path.join(get_package_share_directory(
                    'rae_gazebo'),'config', 'gz_bridge_rae.yaml')
            }],
            output='screen'
        ),

        # Ignition Gazebo - Spawn Entity
        Node(
            package='ros_gz_sim',
            namespace=LaunchConfiguration('namespace'),
            executable='create',
            arguments=[
                "-name", rae_name,
                "-allow_renaming", "true",
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y'),
            ],
            output='screen'
        ),

        # Activate diff controller
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('namespace'),
            arguments=['diff_controller',
                       '--controller-manager', f'{ns_srt}/controller_manager'],
        ),

        # Activate joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('namespace'),
            arguments=['joint_state_broadcaster',
                       '--controller-manager', f'{ns_srt}/controller_manager'],
        ),

        # Activate EKF
        Node(
            condition=IfCondition(enable_localization),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'use_sim_time': True},
                        os.path.join(get_package_share_directory(
                        'rae_hw'), 'config', 'ekf.yaml')],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
            output='screen',
        ),

        # Set all nodes to use simulation time
        SetParameter(name='use_sim_time', value=True)
    ]



def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('enable_localization', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )