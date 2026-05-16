cmake_minimum_required(VERSION 3.8)
project(picar2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  picar2_bringup/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
