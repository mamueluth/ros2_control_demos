cmake_minimum_required(VERSION 3.5)
project(trial_rrbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
