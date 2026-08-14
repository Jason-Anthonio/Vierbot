cmake_minimum_required(VERSION 3.8)
project(diffdrive_arduino)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/diffbot_system.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  hardware/include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  DIRECTORY bringup
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
