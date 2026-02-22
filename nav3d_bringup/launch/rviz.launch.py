import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('nav3d_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav3d.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)
    return ld