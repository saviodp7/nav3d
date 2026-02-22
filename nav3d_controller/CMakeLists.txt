cmake_minimum_required(VERSION 3.5)
project(nav3d_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav3d_core REQUIRED)
find_package(nav3d_common REQUIRED)
find_package(nav3d_msgs REQUIRED)
find_package(nav3d_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav3d_package()

set(executable_name controller_server)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/controller_server.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  angles::angles
  nav3d_core::nav3d_core
  ${nav3d_msgs_TARGETS}
  nav3d_util::nav3d_util_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE
  rclcpp::rclcpp
  ${library_name}
)

# add_library(pose_progress_checker SHARED plugins/pose_progress_checker.cpp)
# target_include_directories(pose_progress_checker
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# target_link_libraries(pose_progress_checker PUBLIC
#   ${geometry_msgs_TARGETS}
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   ${rcl_interfaces_TARGETS}
# )
# target_link_libraries(pose_progress_checker PRIVATE
#   angles::angles
#   nav3d_util::nav3d_util_core
#   pluginlib::pluginlib
# )

add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
target_include_directories(simple_goal_checker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(simple_goal_checker PUBLIC
  ${geometry_msgs_TARGETS}
  nav3d_core::nav3d_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(simple_goal_checker PRIVATE
  angles::angles
  nav3d_util::nav3d_util_core
  pluginlib::pluginlib
  tf2::tf2
)

# add_library(stopped_goal_checker SHARED plugins/stopped_goal_checker.cpp)
# target_include_directories(stopped_goal_checker
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# target_link_libraries(stopped_goal_checker PUBLIC
#   ${geometry_msgs_TARGETS}
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   ${rcl_interfaces_TARGETS}
#   simple_goal_checker
# )
# target_link_libraries(stopped_goal_checker PRIVATE
#   nav3d_util::nav3d_util_core
#   pluginlib::pluginlib
# )

# add_library(position_goal_checker SHARED plugins/position_goal_checker.cpp)
# target_include_directories(position_goal_checker
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# target_link_libraries(position_goal_checker PUBLIC
#   ${geometry_msgs_TARGETS}
#   nav3d_core::nav3d_core
#   # nav3d_costmap_2d::nav3d_costmap_2d_core
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   ${rcl_interfaces_TARGETS}
# )
# target_link_libraries(position_goal_checker PRIVATE
#   angles::angles
#   nav3d_util::nav3d_util_core
#   pluginlib::pluginlib
# )

rclcpp_components_register_nodes(${library_name} "nav3d_controller::ControllerServer")

install(TARGETS simple_goal_checker ${library_name} # position_goal_checker pose_progress_checker stopped_goal_checker
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
ament_export_libraries(
  # pose_progress_checker
  simple_goal_checker
  # stopped_goal_checker
  # position_goal_checker
  ${library_name}
)
ament_export_dependencies(
  geometry_msgs
  nav3d_core
  nav3d_msgs
  nav3d_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  tf2
  tf2_ros
)
ament_export_targets(${library_name})
pluginlib_export_plugin_description_file(nav3d_core plugins.xml)

ament_package()