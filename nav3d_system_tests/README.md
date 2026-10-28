# Nav3D system tests

System tests for Nav3D in simulation.

## Takeoff test

`takeoff_test` starts PX4, Gazebo's `default` world, and Nav3D. It verifies automatic takeoff of at least 0.8 m relative to the initial pose and stable hover near the initial X/Y/yaw, without sending a navigation goal. It writes the result to a JSON report.
