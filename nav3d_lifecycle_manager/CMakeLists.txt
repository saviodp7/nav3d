cmake_minimum_required(VERSION 3.5)
project(nav3d_lifecycle_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav3d_common REQUIRED)
find_package(nav3d_msgs REQUIRED)
find_package(nav3d_util REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)

nav3d_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

set(dependencies
  geometry_msgs
  lifecycle_msgs
  nav3d_msgs
  nav3d_util
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  tf2_geometry_msgs
  bondcpp
  diagnostic_updater
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(lifecycle_manager
  src/main.cpp
)

target_link_libraries(lifecycle_manager
  ${library_name}
)

ament_target_dependencies(lifecycle_manager
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "nav3d_lifecycle_manager::LifecycleManager")

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()