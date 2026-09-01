from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    workspace_root = LaunchConfiguration("workspace_root")
    report_file = LaunchConfiguration("report_file")
    headless = LaunchConfiguration("headless")
    scenario_params = PathJoinSubstitution(
        [
            FindPackageShare("nav3d_system_tests"),
            "scenarios",
            "takeoff_and_navigate",
            "takeoff_and_navigate.params.yaml",
        ]
    )

    px4_process = ExecuteProcess(
        cmd=["bash", PathJoinSubstitution([workspace_root, "scripts", "run_px4_sim.sh"])],
        name="nav3d_test_px4_sim",
        output="log",
        sigterm_timeout="10",
        sigkill_timeout="5",
    )

    nav3d_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav3d_bringup"), "launch", "bringup.launch.py"])
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    test_node = Node(
        package="nav3d_system_tests",
        executable="takeoff_and_navigate_test.py",
        name="takeoff_and_navigate_test",
        output="screen",
        parameters=[
            scenario_params,
            {
                "use_sim_time": True,
                "report_file": report_file,
            }
        ],
        on_exit=[EmitEvent(event=Shutdown(reason="takeoff-and-navigate test completed"))],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workspace_root",
                default_value="/workspaces/nav3d_sim",
                description="Root of the nav3d_sim workspace inside the dev container",
            ),
            DeclareLaunchArgument(
                "report_file",
                default_value=PathJoinSubstitution(
                    [workspace_root, "ros2_ws", "log", "system_tests", "takeoff_and_navigate.json"]
                ),
                description="JSON report written by the system test",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="1",
                description="Run Gazebo without its graphical client",
            ),
            SetEnvironmentVariable("HEADLESS", headless),
            SetEnvironmentVariable("PX4_DAEMON", "1"),
            px4_process,
            TimerAction(period=5.0, actions=[nav3d_bringup]),
            TimerAction(period=8.0, actions=[test_node]),
        ]
    )
