cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

######### Set Dependencies
set(DEPENDENCIES
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

######### Find Dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

######### Robot Hardware Interface
add_library(
  ${PROJECT_NAME}
  SHARED
  odrive_base/src/socket_can.cpp
  odrive_base/src/epoll_event_loop.cpp
  hardware/robot_hardware_interface.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PRIVATE hardware/include)
target_include_directories(${PROJECT_NAME} PRIVATE odrive_base/include)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(hardware_interface odrive_ros2_control_example.xml)
install(
  TARGETS odrive_ros2_control_example
  EXPORT export_odrive_ros2_control_example
)
install(
  DIRECTORY hardware/include 
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY odrive_base/include 
  DESTINATION share/${PROJECT_NAME}/
)

######### Install
# Add description files
install(
  DIRECTORY description/launch description/urdf description/ros2_control
  DESTINATION share/${PROJECT_NAME}/
)
# Add config files
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}/
)
# Add launch files
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
