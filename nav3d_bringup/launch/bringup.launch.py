import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    bringup_dir = get_package_share_directory("nav3d_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    params_file = os.path.join(bringup_dir, "params", "nav3d_params.yaml")

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    ld.add_action(stdout_linebuf_envvar)

    with open(params_file) as file:
        params = yaml.safe_load(file) or {}

    # slam = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("slam", False)
    # map_filepath = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("map", "")
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
    use_sim_time = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("use_sim_time", True)
    use_rviz = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("use_rviz", True)
    rviz_config_file = params.get("nav3d_launcher", {}).get("ros__parameters", {}).get("rviz_config_file", "nav3d.rviz")

    description_pkg = [TextSubstitution(text=robot_name), TextSubstitution(text="_description")]
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare(description_pkg),
            "urdf",
            TextSubstitution(text=f"{robot_name}.urdf.xacro"),
        ]
    )
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": use_sim_time}],
    )
    ld.add_action(start_robot_state_publisher_cmd)
    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": use_sim_time}],
    )
    ld.add_action(start_joint_state_publisher_cmd)

    # === SIMULATION LAYER ===
    gz_bridge_dir = get_package_share_directory("nav3d_gz_bridge")
    start_gz_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_bridge_dir, "launch", "bridge.launch.py")),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )
    ld.add_action(start_gz_bridge_cmd)

    if use_rviz:
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz.launch.py")),
            launch_arguments={
                "use_sim_time": str(use_sim_time),
                "rviz_config": os.path.join(bringup_dir, "rviz", rviz_config_file),
                "namespace": namespace,
            }.items(),
        )
        ld.add_action(rviz_cmd)
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "robot_name": robot_name,
            "use_sim_time": str(use_sim_time),
            # "params_file": params_file,
        }.items(),
    )
    ld.add_action(start_navigation_cmd)

    return ld
