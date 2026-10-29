"""Start the PX4 DDS agent, flight controller and adapter."""

import os
import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter

DDS_PORT = "8888"


def _start_sitl(context):
    px4_dir = os.environ.get("PX4_AUTOPILOT_DIR")
    if not px4_dir:
        workspace_root = os.environ.get("NAV3D_WORKSPACE_ROOT")
        if not workspace_root:
            raise RuntimeError("Set PX4_AUTOPILOT_DIR or NAV3D_WORKSPACE_ROOT to start PX4 SITL")
        px4_dir = str(Path(workspace_root) / "PX4-Autopilot")

    px4_build = Path(px4_dir) / "build" / "px4_sitl_default"
    px4_binary = px4_build / "bin" / "px4"
    px4_rootfs = px4_build / "rootfs"
    if not px4_binary.is_file() or not os.access(px4_binary, os.X_OK) or not px4_rootfs.is_dir():
        raise RuntimeError(f"PX4 SITL must be built before launch: {px4_binary}")

    px4_param = px4_binary.with_name("px4-param")
    if not px4_param.is_file() or not os.access(px4_param, os.X_OK):
        raise RuntimeError(f"PX4 parameter client not found: {px4_param}")

    px4_sitl = ExecuteProcess(
        cmd=[str(px4_binary), "-d"],
        cwd=str(px4_rootfs),
        name="px4_sitl",
        output="screen",
        additional_env={
            "GZ_IP": "127.0.0.1",
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_WORLD": LaunchConfiguration("world").perform(context),
            "PX4_GZ_MODEL_NAME": LaunchConfiguration("uav_name").perform(context),
            "PX4_SIM_MODEL": "gz_x500_depth",
            "PX4_PARAM_EKF2_GPS_CTRL": "0",
            "PX4_PARAM_SENS_EN_GPSSIM": "0",
            "PX4_PARAM_EKF2_HGT_REF": "3",
            "PX4_PARAM_EKF2_EV_CTRL": "3",
            "PX4_UXRCE_DDS_PORT": DDS_PORT,
        },
        on_exit=[EmitEvent(event=Shutdown(reason="PX4 SITL exited"))],
    )

    def set_nav_dll_act(_context):
        try:
            result = subprocess.run(
                [str(px4_param), "set", "NAV_DLL_ACT", "0"],
                cwd=px4_rootfs,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return [EmitEvent(event=Shutdown(reason=f"PX4 NAV_DLL_ACT override failed: {exc}"))]
        if result.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f"PX4 NAV_DLL_ACT override exited {result.returncode}"))]
        return []

    # The X500 airframe changes this default after PX4_PARAM_* values are processed.
    # The startup-complete event ensures the override runs after that change.
    startup_marker = b"Startup script returned successfully"
    startup_output = {"complete": False, "tail": b""}

    def on_px4_output(event):
        if startup_output["complete"]:
            return []
        output = startup_output["tail"] + event.text
        if startup_marker in output:
            startup_output["complete"] = True
            return [OpaqueFunction(function=set_nav_dll_act)]
        startup_output["tail"] = output[-(len(startup_marker) - 1) :]
        return []

    return [
        RegisterEventHandler(OnProcessIO(target_action=px4_sitl, on_stdout=on_px4_output, on_stderr=on_px4_output)),
        px4_sitl,
    ]


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    sitl_started = {"value": False}
    params_file = Path(get_package_share_directory("nav3d_bringup")) / "params" / "nav3d_params.yaml"
    with params_file.open(encoding="utf-8") as stream:
        adapter_params = yaml.safe_load(stream)["nav3d_px4_adapter"]["ros__parameters"]

    def on_spawn_exit(event, _context):
        try:
            is_spawn = isinstance(event.action, Node) and event.action.node_name.rsplit("/", 1)[-1] == "spawn_uav"
        except RuntimeError:
            return []
        if not is_spawn or event.returncode != 0 or sitl_started["value"]:
            return []
        sitl_started["value"] = True
        return [OpaqueFunction(function=_start_sitl)]

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_simulation", default_value="false"),
            DeclareLaunchArgument("world", default_value="default"),
            DeclareLaunchArgument("uav_name", default_value="x500"),
            DeclareLaunchArgument("log_level", default_value="info"),
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", DDS_PORT],
                name="px4_dds_agent",
                output="screen",
                sigterm_timeout="10",
                sigkill_timeout="5",
                on_exit=[EmitEvent(event=Shutdown(reason="PX4 DDS agent exited"))],
            ),
            RegisterEventHandler(
                OnProcessExit(on_exit=on_spawn_exit),
                condition=IfCondition(LaunchConfiguration("use_simulation")),
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(namespace),
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    Node(
                        package="nav3d_px4_adapter",
                        executable="nav3d_px4_adapter",
                        name="nav3d_px4_adapter",
                        output="screen",
                        parameters=[adapter_params],
                        arguments=["--ros-args", "--log-level", log_level],
                    ),
                ]
            ),
        ]
    )
