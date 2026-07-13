cmake_minimum_required(VERSION 3.8)
project(mobile_arm_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf config scripts maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/block_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
