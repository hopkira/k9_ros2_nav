cmake_minimum_required(VERSION 3.8)
project(k9_ros2_nav)
find_package(ament_cmake REQUIRED)
install(FILES
  commissioning/ld06_bringup/ld06.launch.py
  commissioning/ld06_bringup/ld06_filter.launch.py
  commissioning/ld06_bringup/filter_scan.py
  commissioning/ld06_bringup/ld06.yaml
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY commissioning/oak_bringup/
  DESTINATION share/${PROJECT_NAME}/oak
  FILES_MATCHING PATTERN "*.py" PATTERN "*.yaml")
ament_package()
