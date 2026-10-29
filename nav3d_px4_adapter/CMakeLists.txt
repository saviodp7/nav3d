cmake_minimum_required(VERSION 3.8)
project(nav3d_px4_adapter)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav3d_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(nav3d_px4_adapter src/offboard_control.cpp src/vio_odometry_bridge.cpp src/offboard_control_node.cpp)
target_include_directories(
    nav3d_px4_adapter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_link_libraries(nav3d_px4_adapter Eigen3::Eigen)
ament_target_dependencies(
    nav3d_px4_adapter
    rclcpp
    nav3d_msgs
    nav_msgs
    px4_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
)

install(TARGETS nav3d_px4_adapter DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
