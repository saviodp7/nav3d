# nav3d_uavs

ROS 2 package containing the reference UAV models used by Nav3D. Each vehicle has its own directory under `share/nav3d_uavs`.

The X500 files are installed under `x500/description` (URDF, meshes and RViz), `x500/gz/models` (Gazebo SDF models) and `x500/moveit` (MoveIt configuration). Add `share/nav3d_uavs/x500/gz/models` to `GZ_SIM_RESOURCE_PATH` before starting Gazebo. The Gazebo model name is `nav3d_x500`; it does not override PX4's `x500` model.
