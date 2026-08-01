cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  meshes
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_lidar_publisher.py
  scripts/fake_imu_publisher.py
  scripts/spin_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
