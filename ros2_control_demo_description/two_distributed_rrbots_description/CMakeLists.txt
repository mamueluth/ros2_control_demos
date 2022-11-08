cmake_minimum_required(VERSION 3.5)
project(two_distributed_rrbots_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
