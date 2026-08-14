cmake_minimum_required(VERSION 3.8)
project(boat_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fast_launch test/test_fast_launch.py)
  ament_add_pytest_test(test_record_replay test/test_record_replay.py)
endif()

ament_package()
