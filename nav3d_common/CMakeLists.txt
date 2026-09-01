cmake_minimum_required(VERSION 3.5)

project(nav3d_common NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(nav3d_common)

ament_package(CONFIG_EXTRAS "nav3d_common-extras.cmake")

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
