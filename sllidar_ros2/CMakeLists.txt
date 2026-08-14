cmake_minimum_required(VERSION 3.8)
project(sllidar_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  sdk/include
  sdk/src
)

add_executable(sllidar_node
  src/sllidar_node.cpp
)
ament_target_dependencies(sllidar_node
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS
  sllidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
