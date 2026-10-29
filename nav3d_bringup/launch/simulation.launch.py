"""Start a Gazebo world and spawn a Nav3D UAV"""

import os
from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_simulation(context):
    arguments = {
        "world": LaunchConfiguration("world").perform(context),
        "uav_package": LaunchConfiguration("uav_package").perform(context),
        "uav_model": LaunchConfiguration("uav_model").perform(context),
        "uav_name": LaunchConfiguration("uav_name").perform(context),
        "x": LaunchConfiguration("x").perform(context),
        "y": LaunchConfiguration("y").perform(context),
        "z": LaunchConfiguration("z").perform(context),
        "yaw": LaunchConfiguration("yaw").perform(context),
        "gui": LaunchConfiguration("gui").perform(context),
    }
    gui = arguments["gui"].lower()

    world_share = Path(get_package_share_directory("nav3d_gz_worlds"))
    world_file = world_share / "worlds" / f"{arguments['world']}.sdf"
    if not world_file.is_file():
        raise RuntimeError(f"Gazebo world not found: {world_file}")
    world_element = ElementTree.parse(world_file).getroot().find("world")
    if world_element is None or world_element.get("name") != arguments["world"]:
        raise RuntimeError(f"World name in {world_file} must match {arguments['world']!r}")

    uav_share = Path(get_package_share_directory(arguments["uav_package"]))
    model_files = list(uav_share.glob(f"**/models/{arguments['uav_model']}/model.sdf"))
    if len(model_files) != 1:
        raise RuntimeError(
            f"Expected exactly one {arguments['uav_model']}/model.sdf in {uav_share}; found {len(model_files)}"
        )
    model_file = model_files[0]

    resource_paths = [str(world_share / "models"), str(model_file.parent.parent)]
    resource_paths.extend(filter(None, os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep)))
    environment = {"GZ_SIM_RESOURCE_PATH": os.pathsep.join(dict.fromkeys(resource_paths))}

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", *(["-s"] if gui == "false" else []), str(world_file)],
        output="screen",
        additional_env=environment,
        on_exit=[EmitEvent(event=Shutdown(reason="Gazebo exited"))],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_uav",
        output="screen",
        additional_env=environment,
        arguments=[
            "-world",
            arguments["world"],
            "-file",
            str(model_file),
            "-name",
            arguments["uav_name"],
            "-allow_renaming",
            "false",
            "-x",
            arguments["x"],
            "-y",
            arguments["y"],
            "-z",
            arguments["z"],
            "-Y",
            arguments["yaw"],
        ],
    )

    spawn_finished = {"value": False}

    def on_spawn_exit(event, _context):
        spawn_finished["value"] = True
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f"UAV spawn failed with exit code {event.returncode}"))]
        return []

    def on_spawn_timeout(_context):
        if not spawn_finished["value"]:
            return [EmitEvent(event=Shutdown(reason="Timed out waiting for Gazebo UAV spawn"))]
        return []

    return [
        gazebo,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=on_spawn_exit)),
        spawn,
        TimerAction(period=60.0, actions=[OpaqueFunction(function=on_spawn_timeout)]),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("world", default_value="default"),
        DeclareLaunchArgument("uav_package", default_value="nav3d_uavs"),
        DeclareLaunchArgument("uav_model", default_value="nav3d_x500"),
        DeclareLaunchArgument("uav_name", default_value="x500"),
        DeclareLaunchArgument("x", default_value="0"),
        DeclareLaunchArgument("y", default_value="0"),
        DeclareLaunchArgument("z", default_value="0"),
        DeclareLaunchArgument("yaw", default_value="0"),
        DeclareLaunchArgument("gui", default_value="true"),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_simulation)])
