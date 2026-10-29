from uuid import uuid4

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def configure_headless(context):
    value = LaunchConfiguration("headless").perform(context).strip().lower()
    return [SetLaunchConfiguration("gui", "false" if value in ("1", "true") else "true")]


def generate_launch_description() -> LaunchDescription:
    workspace_root = LaunchConfiguration("workspace_root")
    report_file = LaunchConfiguration("report_file")
    scenario_params = PathJoinSubstitution(
        [
            FindPackageShare("nav3d_system_tests"),
            "scenarios",
            "takeoff_test",
            "takeoff_test.params.yaml",
        ]
    )

    bag_output = PathJoinSubstitution([workspace_root, "log", "system_tests", f"takeoff_test_bag_{uuid4().hex}"])
    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            bag_output,
            "/odom",
            "/fmu/out/vehicle_status_v1",
            "/fmu/in/trajectory_setpoint",
        ],
        name="takeoff_bag_recorder",
        output="log",
        sigterm_timeout="10",
        sigkill_timeout="5",
    )

    nav3d_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav3d_bringup"), "launch", "bringup.launch.py"])
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_px4": "true",
            "world": "default",
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    test_node = Node(
        package="nav3d_system_tests",
        executable="takeoff_test.py",
        name="takeoff_test",
        output="screen",
        parameters=[
            scenario_params,
            {"use_sim_time": True, "report_file": report_file},
        ],
        on_exit=[EmitEvent(event=Shutdown(reason="automatic-takeoff test completed"))],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workspace_root",
                default_value="/ros2_ws/src/nav3d",
                description="Root of the Nav3D repository inside the dev container",
            ),
            DeclareLaunchArgument(
                "report_file",
                default_value=PathJoinSubstitution([workspace_root, "log", "system_tests", "takeoff_test.json"]),
                description="JSON report written by the system test",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its graphical client (true in CI)",
            ),
            OpaqueFunction(function=configure_headless),
            bag_recorder,
            # Subscribe before bringup can arm the vehicle, so the ground pose is captured.
            test_node,
            nav3d_bringup,
        ]
    )
