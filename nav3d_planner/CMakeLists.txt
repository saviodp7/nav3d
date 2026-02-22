cmake_minimum_required(VERSION 3.5)
project(nav3d_planner)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav3d_common REQUIRED)
find_package(nav3d_core REQUIRED)
find_package(nav3d_msgs REQUIRED)
find_package(nav3d_util REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(pluginlib REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav3d_package()

set(executable_name planner_server)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/planner_server.cpp
)

target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${library_name}
  geometry_msgs
  nav3d_core
  nav3d_msgs
  nav3d_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${executable_name}
  rclcpp
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "nav3d_planner::PlannerServer")

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav3d_core
  nav3d_msgs
  nav3d_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_export_targets(${library_name})

ament_package()