<h1 align="center">Nav3D</h1>

<p align="center">
  <strong>A Nav2-inspired 3D navigation stack for UAVs</strong>
</p>

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-22314E">
  <img alt="PX4 SITL" src="https://img.shields.io/badge/PX4-SITL-1B6AC6">
  <img alt="MoveIt 2" src="https://img.shields.io/badge/MoveIt-2-68A063">
  <img alt="Gazebo Garden" src="https://img.shields.io/badge/Gazebo-Garden-8A2BE2">
</p>

Nav3D is a ROS 2 navigation stack that reformulates the Nav2 architectural model for
three-dimensional UAV navigation. It keeps the parts of Nav2 that are valuable as a
navigation framework, such as action-based task interfaces, lifecycle-managed servers,
plugin-oriented planning and Behavior Tree execution, while replacing the planar costmap
assumption with a 3D collision-aware pipeline based on MoveIt, OctoMap and PX4 offboard
setpoint streaming.

The project originates from the master's thesis *Designing a Nav2-Based 3D Navigation
Stack for UAVs*. The thesis motivation is practical: ROS 2 has a mature, modular
navigation framework for ground robots, but UAV navigation commonly appears as tightly
coupled planning pipelines or platform-specific systems. Nav3D explores whether a
Nav2-like stack can be preserved at the architectural level while making mapping,
planning, validation and execution consistent with volumetric flight.

## Research Goals

Nav3D is designed around four goals:

- Preserve a Nav2-style separation between task coordination, global planning and path
  execution.
- Plan in full 3D using a volumetric world model instead of projecting the environment
  onto a 2D costmap.
- Keep the navigation core independent from a specific airframe by isolating robot model,
  TF and MoveIt configuration in a dedicated integration package.
- Execute geometric 3D paths through a flight-controller-friendly interface, delegating
  low-level stabilization to PX4 while Nav3D manages path progression, validity and goal
  completion.

## Main Features

- **Behavior Tree navigation** with replanning on goal updates or path invalidation.
- **Lifecycle-managed planner, controller and BT navigator servers**, following the
  operational structure used by Nav2.
- **MoveIt-based 3D global planner plugin** that adapts a navigation goal into an SE(3)
  planning request for a floating-base UAV.
- **OctoMap-backed PlanningScene integration** through depth point-cloud updates and
  MoveIt collision checking.
- **Local path validity checks** that query the same 3D planning scene used by the global
  planner.
- **Waypoint-streaming controller** that converts a geometric path into time-paced
  `nav3d_msgs/TrajectoryPoint` references.
- **PX4 adapter** that converts ROS ENU references into PX4-compatible NED
  trajectory setpoints.
- **Gazebo/PX4 SITL reference setup** for an X500 UAV equipped with depth perception.

## Architecture

At runtime, a navigation request is received as a 3D pose. The BT Navigator coordinates
planning and execution through the selected Behavior Tree. The planner server calls the
configured global planner plugin, currently `MoveItGlobalPlanner`, which generates a
collision-free geometric path in the current MoveIt PlanningScene. The controller server
then streams time-paced waypoint references along that path. The PX4 adapter performs the
ENU-to-NED conversion and publishes PX4 `TrajectorySetpoint` messages in offboard mode.

The default Behavior Tree checks a truncated local portion of the active path. If that
segment becomes invalid after the map is updated, the tree triggers replanning and the new
path is passed back to the controller:

```text
Goal -> ComputePathToPose -> FollowPath
                ^                |
                |                v
        IsPathValid on local remaining path
```

## Packages

| Package | Role |
| --- | --- |
| `nav3d_bringup` | Launch files, RViz config and runtime parameters. |
| `nav3d_bt_navigator` | Behavior Tree based navigation action server. |
| `nav3d_behavior_tree` | BT action, condition, decorator and utility nodes. |
| `nav3d_planner` | Planner server exposing `ComputePathToPose` and path validity services. |
| `nav3d_planner_plugin_moveit` | MoveIt/OMPL global planner plugin for 3D path generation and validity queries. |
| `nav3d_controller` | Path execution server and waypoint-streaming controller. |
| `nav3d_px4_adapter` | PX4 offboard trajectory-setpoint integration and VIO/odometry bridge. |
| `nav3d_gz_bridge` | Gazebo to ROS 2 topic bridges and static transforms for simulation. |
| `nav3d_lifecycle_manager` | Lifecycle orchestration for the navigation servers. |
| `nav3d_core` | Core plugin interfaces and shared navigation abstractions. |
| `nav3d_common` | Shared launch and configuration utilities. |
| `nav3d_util` | Lifecycle, action, service, TF and odometry utilities. |
| `nav3d_msgs` | Actions, services and messages used by the stack. |

The reference airframe integration is provided by `x500_nav3d`, which contains the X500
description and the MoveIt configuration for the floating-base planning group. Keeping this
outside the navigation core is intentional: a different UAV should be integrated by
providing an equivalent robot description, TF structure and MoveIt configuration.

## Reference Stack

The reference setup used during development and evaluation is:

- Ubuntu 22.04 with ROS 2 Humble.
- Gazebo Garden through `ros_gz`.
- PX4 SITL with Micro XRCE-DDS communication.
- X500 depth-camera model.
- MoveIt 2 with OMPL


## Quick Start

The easiest way to reproduce the reference setup is to use the full `nav3d_sim` workspace
with its devcontainer. The devcontainer installs ROS 2 Humble, Gazebo Garden, MoveIt 2,
PX4 dependencies and the Micro XRCE-DDS Agent.

Build the ROS 2 workspace:

```bash
cd /root/nav3d_sim/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Start PX4 SITL and the XRCE-DDS agent in one terminal:

```bash
cd /root/nav3d_sim
./scripts/run_px4_sim.sh
```

Start Nav3D in another terminal:

```bash
cd /root/nav3d_sim/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav3d_bringup bringup.launch.py
```

Send a sample navigation goal:

```bash
ros2 action send_goal /navigate_to_pose nav3d_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}, behavior_tree: ''}"
```

## Standalone Integration Notes

If this package is used outside the original simulation workspace, the following components
must be provided by the surrounding ROS 2 workspace:

- `px4_msgs` compatible with the PX4 version used in simulation or flight.
- A UAV description package publishing a valid `robot_description` and TF tree.
- A MoveIt configuration package with a floating-base planning group.
- A depth point-cloud source if online OctoMap updates are required.
- A bridge or controller that consumes `nav3d_msgs/TrajectoryPoint` if PX4 is not used.

The navigation core should not require airframe-specific changes as long as the robot
integration package preserves the expected frames, planning group and topic contracts.

## License

Nav3D is released under the Apache License, Version 2.0. See [LICENSE](LICENSE).
Redistributions and derivative works must preserve the license terms and the attribution
notices listed in [NOTICE](NOTICE).
