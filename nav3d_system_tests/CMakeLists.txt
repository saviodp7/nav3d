cmake_minimum_required(VERSION 3.8)
project(nav3d_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  install(
    PROGRAMS scenarios/takeoff_test/takeoff_test.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME takeoff_test.py
  )

  install(
    PROGRAMS scripts/run_system_test
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    FILES scenarios/takeoff_test/takeoff_test.launch.py
    DESTINATION share/${PROJECT_NAME}/launch
  )

  install(
    DIRECTORY scenarios
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING
    PATTERN "*.yaml"
  )
endif()

ament_package()
