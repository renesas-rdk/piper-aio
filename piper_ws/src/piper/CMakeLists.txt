cmake_minimum_required(VERSION 3.10)
project(piper)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/piper_start_leader_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
