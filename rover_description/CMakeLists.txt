cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  meshes
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_lidar_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
