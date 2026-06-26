cmake_minimum_required(VERSION 3.8)
project(mobile_arm_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf config scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
