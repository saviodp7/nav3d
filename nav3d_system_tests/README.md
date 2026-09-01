# Nav3D system tests

This package contains scenario-based, full-system integration tests for Nav3D.
Each scenario may start PX4 SITL, Gazebo, Nav3D bringup, and a test node that
checks the resulting vehicle behavior. Unit tests remain in the package that
owns the component under test.

## Scenarios

### `takeoff_and_navigate`

Starts the simulation headlessly, waits for Nav3D and PX4 readiness, verifies a
stable takeoff, sends one `NavigateToPose` goal, and writes a machine-readable
JSON report.

The scenario owns its launch file, parameters, and test implementation under:

```text
scenarios/takeoff_and_navigate/
```

## Build

```bash
colcon build --symlink-install --packages-select nav3d_bringup nav3d_system_tests
source install/setup.bash
```

System tests are installed only when `BUILD_TESTING` is enabled, which is the
default for a normal development build.

## Run

Run the default scenario:

```bash
ros2 run nav3d_system_tests run_system_test
```

Or name it explicitly and pass launch arguments after the scenario name:

```bash
ros2 run nav3d_system_tests run_system_test takeoff_and_navigate \
  report_file:=/tmp/takeoff_and_navigate.json headless:=1
```

The process exits with zero only when the scenario passes. Its default report
is written to:

```text
/workspaces/nav3d_sim/ros2_ws/log/system_tests/takeoff_and_navigate.json
```
