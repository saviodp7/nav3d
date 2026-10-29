import math
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate_simulation_arguments(context):
    arguments = {
        "world": LaunchConfiguration("world").perform(context),
        "uav_model": LaunchConfiguration("uav_model").perform(context),
        "uav_name": LaunchConfiguration("uav_name").perform(context),
        "x": LaunchConfiguration("x").perform(context),
        "y": LaunchConfiguration("y").perform(context),
        "z": LaunchConfiguration("z").perform(context),
        "yaw": LaunchConfiguration("yaw").perform(context),
        "gui": LaunchConfiguration("gui").perform(context),
        "use_px4": LaunchConfiguration("use_px4").perform(context),
    }
    for name in ("world", "uav_model", "uav_name"):
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", arguments[name]):
            raise RuntimeError(f"Invalid {name}: {arguments[name]!r}")
    for name in ("x", "y", "z", "yaw"):
        try:
            valid = math.isfinite(float(arguments[name]))
        except ValueError:
            valid = False
        if not valid:
            raise RuntimeError(f"Invalid {name}: {arguments[name]!r}")
    if arguments["gui"].lower() not in ("true", "false"):
        raise RuntimeError("gui must be true or false")
    if arguments["use_px4"].lower() not in ("true", "false"):
        raise RuntimeError("use_px4 must be true or false")
    return []


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    bringup_dir = get_package_share_directory("nav3d_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    gz_bridge_dir = get_package_share_directory("nav3d_gz_bridge")
    params_file = os.path.join(bringup_dir, "params", "nav3d_params.yaml")

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    ld.add_action(stdout_linebuf_envvar)

    with open(params_file) as file:
        params = yaml.safe_load(file) or {}

    # slam = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("slam", False)
    # keepout_mask_yaml_file = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("keepout_mask", "")
    # speed_mask_yaml_file = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("speed_mask", "")
    # graph_filepath = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("graph", "")
    # autostart = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("autostart", True)
    # use_respawn = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("use_respawn", False)
    # use_localization = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("use_localization", False)
    # use_keepout_zones = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("use_keepout_zones", False)
    # use_speed_zones = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("use_speed_zones", False)

    namespace = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("namespace", "")
    robot_name = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("robot_name", "x500")
    default_use_rviz = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("use_rviz", True)
    use_rviz = LaunchConfiguration("use_rviz")
    use_simulation = LaunchConfiguration("use_simulation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    uav_name = LaunchConfiguration("uav_name")
    rviz_config_file = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("rviz_config_file", "nav3d.rviz")

    ld.add_action(DeclareLaunchArgument("use_simulation", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value=use_simulation))
    ld.add_action(DeclareLaunchArgument("world", default_value="leo_race_field"))
    ld.add_action(DeclareLaunchArgument("uav_package", default_value="nav3d_uavs"))
    ld.add_action(DeclareLaunchArgument("uav_model", default_value="nav3d_x500"))
    ld.add_action(DeclareLaunchArgument("uav_name", default_value="x500"))
    ld.add_action(DeclareLaunchArgument("x", default_value="0"))
    ld.add_action(DeclareLaunchArgument("y", default_value="0"))
    ld.add_action(DeclareLaunchArgument("z", default_value="0"))
    ld.add_action(DeclareLaunchArgument("yaw", default_value="0"))
    ld.add_action(DeclareLaunchArgument("gui", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_px4", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value=str(default_use_rviz).lower()))
    ld.add_action(OpaqueFunction(function=_validate_simulation_arguments, condition=IfCondition(use_simulation)))

    # === SIMULATION LAYER ===
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "simulation.launch.py")),
            condition=IfCondition(use_simulation),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "uav_package": LaunchConfiguration("uav_package"),
                "uav_model": LaunchConfiguration("uav_model"),
                "uav_name": LaunchConfiguration("uav_name"),
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "yaw": LaunchConfiguration("yaw"),
                "gui": LaunchConfiguration("gui"),
            }.items(),
        )
    )
    start_gz_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_bridge_dir, "launch", "bridge.launch.py")),
        condition=IfCondition(use_simulation),
        launch_arguments={
            "namespace": namespace,
            "uav_name": uav_name,
        }.items(),
    )
    ld.add_action(start_gz_bridge_cmd)

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("nav3d_uavs"),
            robot_name,
            "description",
            "urdf",
            f"{robot_name}.urdf.xacro",
        ]
    )
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )
    ld.add_action(start_robot_state_publisher_cmd)
    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )
    ld.add_action(start_joint_state_publisher_cmd)

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz.launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": os.path.join(bringup_dir, "rviz", rviz_config_file),
            "namespace": namespace,
        }.items(),
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "px4.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_px4")),
            launch_arguments={
                "use_simulation": use_simulation,
                "use_sim_time": use_sim_time,
                "namespace": namespace,
                "world": LaunchConfiguration("world"),
                "uav_name": uav_name,
            }.items(),
        )
    )

    ld.add_action(rviz_cmd)
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "robot_name": robot_name,
            "use_sim_time": use_sim_time,
            # "params_file": params_file,
        }.items(),
    )
    ld.add_action(start_navigation_cmd)

    return ld
