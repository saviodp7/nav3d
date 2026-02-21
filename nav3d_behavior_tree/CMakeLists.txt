cmake_minimum_required(VERSION 3.5)
project(nav3d_behavior_tree CXX)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav3d_common REQUIRED)
find_package(nav3d_msgs REQUIRED)
find_package(nav3d_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav3d_package()

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${library_name} PUBLIC
  ${action_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav3d_msgs_TARGETS}
  nav3d_util::nav3d_util_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

add_library(nav3d_compute_path_to_pose_action_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
list(APPEND plugin_libs nav3d_compute_path_to_pose_action_bt_node)

add_library(nav3d_compute_path_through_poses_action_bt_node SHARED plugins/action/compute_path_through_poses_action.cpp)
list(APPEND plugin_libs nav3d_compute_path_through_poses_action_bt_node)

add_library(nav3d_controller_cancel_bt_node SHARED plugins/action/controller_cancel_node.cpp)
list(APPEND plugin_libs nav3d_controller_cancel_bt_node)

add_library(nav3d_wait_cancel_bt_node SHARED plugins/action/wait_cancel_node.cpp)
list(APPEND plugin_libs nav3d_wait_cancel_bt_node)

add_library(nav3d_spin_cancel_bt_node SHARED plugins/action/spin_cancel_node.cpp)
list(APPEND plugin_libs nav3d_spin_cancel_bt_node)

add_library(nav3d_back_up_cancel_bt_node SHARED plugins/action/back_up_cancel_node.cpp)
list(APPEND plugin_libs nav3d_back_up_cancel_bt_node)

add_library(nav3d_assisted_teleop_cancel_bt_node SHARED plugins/action/assisted_teleop_cancel_node.cpp)
list(APPEND plugin_libs nav3d_assisted_teleop_cancel_bt_node)

add_library(nav3d_drive_on_heading_cancel_bt_node SHARED plugins/action/drive_on_heading_cancel_node.cpp)
list(APPEND plugin_libs nav3d_drive_on_heading_cancel_bt_node)

add_library(nav3d_smooth_path_action_bt_node SHARED plugins/action/smooth_path_action.cpp)
list(APPEND plugin_libs nav3d_smooth_path_action_bt_node)

add_library(nav3d_follow_path_action_bt_node SHARED plugins/action/follow_path_action.cpp)
list(APPEND plugin_libs nav3d_follow_path_action_bt_node)

add_library(nav3d_back_up_action_bt_node SHARED plugins/action/back_up_action.cpp)
list(APPEND plugin_libs nav3d_back_up_action_bt_node)

add_library(nav3d_drive_on_heading_bt_node SHARED plugins/action/drive_on_heading_action.cpp)
list(APPEND plugin_libs nav3d_drive_on_heading_bt_node)

add_library(nav3d_spin_action_bt_node SHARED plugins/action/spin_action.cpp)
list(APPEND plugin_libs nav3d_spin_action_bt_node)

add_library(nav3d_wait_action_bt_node SHARED plugins/action/wait_action.cpp)
list(APPEND plugin_libs nav3d_wait_action_bt_node)

add_library(nav3d_assisted_teleop_action_bt_node SHARED plugins/action/assisted_teleop_action.cpp)
list(APPEND plugin_libs nav3d_assisted_teleop_action_bt_node)

add_library(nav3d_is_stuck_condition_bt_node SHARED plugins/condition/is_stuck_condition.cpp)
list(APPEND plugin_libs nav3d_is_stuck_condition_bt_node)

add_library(nav3d_transform_available_condition_bt_node SHARED plugins/condition/transform_available_condition.cpp)
list(APPEND plugin_libs nav3d_transform_available_condition_bt_node)

add_library(nav3d_goal_reached_condition_bt_node SHARED plugins/condition/goal_reached_condition.cpp)
list(APPEND plugin_libs nav3d_goal_reached_condition_bt_node)

add_library(nav3d_are_poses_near_condition_bt_node SHARED plugins/condition/are_poses_near_condition.cpp)
list(APPEND plugin_libs nav3d_are_poses_near_condition_bt_node)

add_library(nav3d_globally_updated_goal_condition_bt_node SHARED plugins/condition/globally_updated_goal_condition.cpp)
list(APPEND plugin_libs nav3d_globally_updated_goal_condition_bt_node)

add_library(nav3d_goal_updated_condition_bt_node SHARED plugins/condition/goal_updated_condition.cpp)
list(APPEND plugin_libs nav3d_goal_updated_condition_bt_node)

add_library(nav3d_time_expired_condition_bt_node SHARED plugins/condition/time_expired_condition.cpp)
list(APPEND plugin_libs nav3d_time_expired_condition_bt_node)

add_library(nav3d_path_expiring_timer_condition SHARED plugins/condition/path_expiring_timer_condition.cpp)
list(APPEND plugin_libs nav3d_path_expiring_timer_condition)

add_library(nav3d_distance_traveled_condition_bt_node SHARED plugins/condition/distance_traveled_condition.cpp)
list(APPEND plugin_libs nav3d_distance_traveled_condition_bt_node)

add_library(nav3d_initial_pose_received_condition_bt_node SHARED plugins/condition/initial_pose_received_condition.cpp)
list(APPEND plugin_libs nav3d_initial_pose_received_condition_bt_node)

add_library(nav3d_is_battery_charging_condition_bt_node SHARED plugins/condition/is_battery_charging_condition.cpp)
list(APPEND plugin_libs nav3d_is_battery_charging_condition_bt_node)

add_library(nav3d_is_battery_low_condition_bt_node SHARED plugins/condition/is_battery_low_condition.cpp)
list(APPEND plugin_libs nav3d_is_battery_low_condition_bt_node)

add_library(nav3d_are_error_codes_active_condition_bt_node SHARED plugins/condition/are_error_codes_present_condition.cpp)
list(APPEND plugin_libs nav3d_are_error_codes_active_condition_bt_node)

add_library(nav3d_would_a_controller_recovery_help_condition_bt_node SHARED plugins/condition/would_a_controller_recovery_help_condition.cpp)
list(APPEND plugin_libs nav3d_would_a_controller_recovery_help_condition_bt_node)

add_library(nav3d_would_a_planner_recovery_help_condition_bt_node SHARED plugins/condition/would_a_planner_recovery_help_condition.cpp)
list(APPEND plugin_libs nav3d_would_a_planner_recovery_help_condition_bt_node)

add_library(nav3d_would_a_smoother_recovery_help_condition_bt_node SHARED plugins/condition/would_a_smoother_recovery_help_condition.cpp)
list(APPEND plugin_libs nav3d_would_a_smoother_recovery_help_condition_bt_node)

add_library(nav3d_would_a_route_recovery_help_condition_bt_node SHARED plugins/condition/would_a_route_recovery_help_condition.cpp)
list(APPEND plugin_libs nav3d_would_a_route_recovery_help_condition_bt_node)

add_library(nav3d_rate_controller_bt_node SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs nav3d_rate_controller_bt_node)

add_library(nav3d_distance_controller_bt_node SHARED plugins/decorator/distance_controller.cpp)
list(APPEND plugin_libs nav3d_distance_controller_bt_node)

add_library(nav3d_speed_controller_bt_node SHARED plugins/decorator/speed_controller.cpp)
list(APPEND plugin_libs nav3d_speed_controller_bt_node)

add_library(nav3d_truncate_path_action_bt_node SHARED plugins/action/truncate_path_action.cpp)
list(APPEND plugin_libs nav3d_truncate_path_action_bt_node)

add_library(nav3d_concatenate_paths_action_bt_node SHARED plugins/action/concatenate_paths_action.cpp)
list(APPEND plugin_libs nav3d_concatenate_paths_action_bt_node)

add_library(nav3d_truncate_path_local_action_bt_node SHARED plugins/action/truncate_path_local_action.cpp)
list(APPEND plugin_libs nav3d_truncate_path_local_action_bt_node)

add_library(nav3d_goal_updater_node_bt_node SHARED plugins/decorator/goal_updater_node.cpp)
list(APPEND plugin_libs nav3d_goal_updater_node_bt_node)

add_library(nav3d_path_longer_on_approach_bt_node SHARED plugins/decorator/path_longer_on_approach.cpp)
list(APPEND plugin_libs nav3d_path_longer_on_approach_bt_node)

add_library(nav3d_recovery_node_bt_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs nav3d_recovery_node_bt_node)

add_library(nav3d_navigate_to_pose_action_bt_node SHARED plugins/action/navigate_to_pose_action.cpp)
list(APPEND plugin_libs nav3d_navigate_to_pose_action_bt_node)

add_library(nav3d_navigate_through_poses_action_bt_node SHARED plugins/action/navigate_through_poses_action.cpp)
list(APPEND plugin_libs nav3d_navigate_through_poses_action_bt_node)

add_library(nav3d_append_goal_pose_to_goals_action_bt_node SHARED plugins/action/append_goal_pose_to_goals_action.cpp)
list(APPEND plugin_libs nav3d_append_goal_pose_to_goals_action_bt_node)

add_library(nav3d_get_pose_from_path_action_bt_node SHARED plugins/action/get_pose_from_path_action.cpp)
list(APPEND plugin_libs nav3d_get_pose_from_path_action_bt_node)

add_library(nav3d_extract_route_nodes_as_goals_action_bt_node SHARED plugins/action/extract_route_nodes_as_goals_action.cpp)
list(APPEND plugin_libs nav3d_extract_route_nodes_as_goals_action_bt_node)

add_library(nav3d_get_next_few_goals_action_bt_node SHARED plugins/action/get_next_few_goals_action.cpp)
list(APPEND plugin_libs nav3d_get_next_few_goals_action_bt_node)

add_library(nav3d_get_current_pose_action_bt_node SHARED plugins/action/get_current_pose_action.cpp)
list(APPEND plugin_libs nav3d_get_current_pose_action_bt_node)

add_library(nav3d_pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs nav3d_pipeline_sequence_bt_node)

add_library(nav3d_round_robin_node_bt_node SHARED plugins/control/round_robin_node.cpp)
list(APPEND plugin_libs nav3d_round_robin_node_bt_node)

add_library(nav3d_single_trigger_bt_node SHARED plugins/decorator/single_trigger_node.cpp)
list(APPEND plugin_libs nav3d_single_trigger_bt_node)

add_library(nav3d_planner_selector_bt_node SHARED plugins/action/planner_selector_node.cpp)
list(APPEND plugin_libs nav3d_planner_selector_bt_node)

add_library(nav3d_controller_selector_bt_node SHARED plugins/action/controller_selector_node.cpp)
list(APPEND plugin_libs nav3d_controller_selector_bt_node)

add_library(nav3d_smoother_selector_bt_node SHARED plugins/action/smoother_selector_node.cpp)
list(APPEND plugin_libs nav3d_smoother_selector_bt_node)

add_library(nav3d_goal_checker_selector_bt_node SHARED plugins/action/goal_checker_selector_node.cpp)
list(APPEND plugin_libs nav3d_goal_checker_selector_bt_node)

add_library(nav3d_progress_checker_selector_bt_node SHARED plugins/action/progress_checker_selector_node.cpp)
list(APPEND plugin_libs nav3d_progress_checker_selector_bt_node)

add_library(nav3d_goal_updated_controller_bt_node SHARED plugins/decorator/goal_updated_controller.cpp)
list(APPEND plugin_libs nav3d_goal_updated_controller_bt_node)

add_library(nav3d_compute_and_track_route_cancel_bt_node SHARED plugins/action/compute_and_track_route_cancel_node.cpp)
list(APPEND plugin_libs nav3d_compute_and_track_route_cancel_bt_node)

add_library(nav3d_compute_and_track_route_bt_node SHARED plugins/action/compute_and_track_route_action.cpp)
list(APPEND plugin_libs nav3d_compute_and_track_route_bt_node)

add_library(nav3d_compute_route_bt_node SHARED plugins/action/compute_route_action.cpp)
list(APPEND plugin_libs nav3d_compute_route_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin}
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
  target_link_libraries(${bt_plugin} PUBLIC
    behaviortree_cpp::behaviortree_cpp
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav3d_msgs_TARGETS}
    nav3d_util::nav3d_util_core
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name} ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)

add_executable(generate_nav3d_tree_nodes_xml src/generate_nav3d_tree_nodes_xml.cpp)
target_link_libraries(generate_nav3d_tree_nodes_xml PRIVATE
  behaviortree_cpp::behaviortree_cpp
  nav3d_util::nav3d_util_core
)
target_include_directories(generate_nav3d_tree_nodes_xml PRIVATE ${GENERATED_DIR})
install(TARGETS generate_nav3d_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES nav3d_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

ament_export_include_directories(
  include/${PROJECT_NAME}
)

ament_export_libraries(${library_name})

ament_export_dependencies(
  action_msgs
  behaviortree_cpp
  geometry_msgs
  nav3d_common
  nav3d_msgs
  nav3d_util
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()