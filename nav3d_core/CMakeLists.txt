cmake_minimum_required(VERSION 3.5)
project(nav3d_core)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav3d_behavior_tree REQUIRED)
find_package(nav3d_common REQUIRED)
find_package(nav3d_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(nav3d_core INTERFACE)
target_include_directories(
    nav3d_core INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(
    nav3d_core
    INTERFACE ${geometry_msgs_TARGETS}
              ${nav_msgs_TARGETS}
              pluginlib::pluginlib
              rclcpp::rclcpp
              rclcpp_lifecycle::rclcpp_lifecycle
              tf2_ros::tf2_ros
              nav3d_behavior_tree::nav3d_behavior_tree
              nav3d_util::nav3d_util_core
)

install(
    TARGETS nav3d_core
    EXPORT nav3d_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if (BUILD_TESTING)

endif ()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
    geometry_msgs
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
    tf2_ros
    nav3d_behavior_tree
    nav3d_util
)
ament_export_targets(nav3d_core)

ament_package()
