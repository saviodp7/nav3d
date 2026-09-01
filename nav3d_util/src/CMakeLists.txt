add_library(
    ${library_name} SHARED
    node_utils.cpp
    lifecycle_service_client.cpp
    string_utils.cpp
    lifecycle_utils.cpp
    lifecycle_node.cpp
    robot_utils.cpp
    node_thread.cpp
    odometry_utils.cpp
    array_parser.cpp
)

target_include_directories(
    ${library_name} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
                           "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(
    ${library_name}
    PUBLIC bondcpp::bondcpp
           ${geometry_msgs_TARGETS}
           ${lifecycle_msgs_TARGETS}
           ${nav3d_msgs_TARGETS}
           ${nav_msgs_TARGETS}
           ${rcl_interfaces_TARGETS}
           rclcpp::rclcpp
           rclcpp_action::rclcpp_action
           rclcpp_lifecycle::rclcpp_lifecycle
           tf2_ros::tf2_ros
           tf2::tf2
           ${tf2_geometry_msgs_TARGETS}
           pluginlib::pluginlib
)

target_link_libraries(${library_name} PRIVATE ${bond_TARGETS})

add_executable(lifecycle_bringup lifecycle_bringup_commandline.cpp)
target_link_libraries(lifecycle_bringup PRIVATE ${library_name} rclcpp::rclcpp)

install(
    TARGETS ${library_name}
    EXPORT export_${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS lifecycle_bringup RUNTIME DESTINATION lib/${PROJECT_NAME})
