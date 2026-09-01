cmake_minimum_required(VERSION 3.5)
project(nav3d_util)

find_package(ament_cmake REQUIRED)
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav3d_common REQUIRED)
find_package(nav3d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

nav3d_package()

set(library_name ${PROJECT_NAME}_core)

add_subdirectory(src)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
    bondcpp
    geometry_msgs
    lifecycle_msgs
    nav3d_msgs
    nav_msgs
    rcl_interfaces
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    pluginlib
)
ament_export_targets(export_${library_name})

ament_package()
