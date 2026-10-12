# Nav3D system tests

This package runs Nav3D system tests in simulation. Unit tests live alongside
the components they test.

## Takeoff test

`takeoff_test` starts PX4, Gazebo's `default` world, and Nav3D. It captures the
disarmed pose, checks that the vehicle climbs at least 0.8 m relative to it,
and briefly hovers while remaining near the initial X/Y/yaw. It then writes a
JSON report. It does not send a navigation goal.

From `/workspaces/nav3d_sim/ros2_ws` in the Dev Container, build the test and
its dependencies:

```bash
colcon build --symlink-install --packages-up-to nav3d_system_tests
source install/setup.bash
```

With no other Gazebo world running, start the default scenario:

```bash
ros2 run nav3d_system_tests run_system_test
```

To choose a report path, name the scenario and pass a launch argument:

```bash
ros2 run nav3d_system_tests run_system_test takeoff_test \
  report_file:=/tmp/takeoff_test.json
```

By default, the report is written to
`/workspaces/nav3d_sim/ros2_ws/log/system_tests/takeoff_test.json`. Check its
`passed` field (`true` or `false`) and `failure` details to see the result.

The 0.8 m minimum climb is a baseline check, not the target altitude for a
future explicit Takeoff behavior. The default horizontal drift limit is 0.5 m
and the yaw error limit is 0.35 rad; both are configurable scenario parameters.
The scenario files are in
`scenarios/takeoff_test/`. Tests are installed when `BUILD_TESTING` is enabled
(the default).
