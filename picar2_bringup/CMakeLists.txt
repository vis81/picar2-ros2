cmake_minimum_required(VERSION 3.8)
project(picar2_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# C++ port of picar2_bringup/scan_deskew.py (kept, same maths); this one
# costs ~2 % of a Pi core where the Python one cost 25.
add_executable(scan_deskew src/scan_deskew.cpp)
ament_target_dependencies(scan_deskew rclcpp sensor_msgs tf2 tf2_ros)
install(TARGETS scan_deskew DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  picar2_bringup/cmd_vel_relay.py
  picar2_bringup/odom_cal.py
  picar2_bringup/imu_verify.py
  picar2_bringup/joy_feedback.py
  picar2_bringup/frontier_explorer.py
  picar2_bringup/explore_map_trinary.py
  picar2_bringup/scan_deskew.py
  picar2_bringup/cpu_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
