import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterFile
from nav3d_common.launch.rewritten_yaml import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    bringup_dir = get_package_share_directory("nav3d_bringup")

    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    params_file = LaunchConfiguration("params_file")

    lifecycle_nodes = ["planner_server", "controller_server", "bt_navigator"]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="x500",
        description="Robot name for description package",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the nodes",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav3d_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    load_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            SetParameter(name="use_sim_time", value=use_sim_time),
            Node(
                package="nav3d_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"autostart": autostart}, {"node_names": lifecycle_nodes}],
            ),
            Node(
                package="nav3d_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav3d_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav3d_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # Node(
            #     package='nav3d_behaviors',
            #     executable='behavior_server',
            #     name='behavior_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            # ),
            Node(
                package="nav3d_px4_bridge",
                executable="vio_odometry_bridge",
                name="vio_odometry_bridge",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav3d_px4_bridge",
                executable="offboard_control_node",
                name="offboard_control",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "movegroup.launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "robot_name": robot_name,
                    "use_sim_time": use_sim_time,
                    "moveit_config_package": [robot_name, TextSubstitution(text="_moveit_config")],
                }.items(),
            ),
        ],
    )

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(load_nodes)

    return ld
