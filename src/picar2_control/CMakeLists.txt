cmake_minimum_required(VERSION 3.8)
project(picar2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(picar2_control SHARED
  src/picar2_hardware.cpp
)
target_include_directories(picar2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(picar2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface picar2_control.xml)

install(TARGETS picar2_control
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES picar2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
