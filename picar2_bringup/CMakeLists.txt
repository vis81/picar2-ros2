cmake_minimum_required(VERSION 3.8)
project(picar2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  picar2_bringup/cmd_vel_relay.py
  picar2_bringup/odom_cal.py
  picar2_bringup/imu_verify.py
  picar2_bringup/joy_feedback.py
  picar2_bringup/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
