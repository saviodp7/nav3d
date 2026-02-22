cmake_minimum_required(VERSION 3.5)
project(nav3d_bringup)

find_package(ament_cmake REQUIRED)
# find_package(nav3d_common REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()
