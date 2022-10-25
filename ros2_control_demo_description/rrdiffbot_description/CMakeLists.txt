cmake_minimum_required(VERSION 3.5)
project(rrdiffbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY gazebo urdf ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
