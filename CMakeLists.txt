cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add launch files
install(
  DIRECTORY description/launch description/urdf description/ros2_control
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
