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
  controller_interface
  generate_parameter_library
  control_toolbox
  control_msgs
  realtime_tools
)

######### Find Dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

######### ODrive Hardware Interface
add_library(
  odrive_hardware_interface
  SHARED
  odrive_base/src/socket_can.cpp
  odrive_base/src/epoll_event_loop.cpp
  hardware/odrive_hardware_interface.cpp
)
target_compile_features(odrive_hardware_interface PUBLIC cxx_std_20)
target_include_directories(odrive_hardware_interface PRIVATE hardware/include)
target_include_directories(odrive_hardware_interface PRIVATE odrive_base/include)
ament_target_dependencies(
  odrive_hardware_interface 
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)
install(TARGETS odrive_hardware_interface EXPORT export_odrive_hardware_interface)
install(DIRECTORY hardware/include DESTINATION share/odrive_hardware_interface)
install(DIRECTORY odrive_base/include DESTINATION share/odrive_hardware_interface)

######### ODrive Controller
generate_parameter_library(odrive_controller_parameters controller/odrive_controller_parameters.yaml)
add_library(
  odrive_controller
  SHARED
  controller/odrive_controller.cpp
)
target_compile_features(odrive_controller PUBLIC cxx_std_20)
target_include_directories(odrive_controller PRIVATE controller/include)
target_link_libraries(odrive_controller PUBLIC odrive_controller_parameters)
ament_target_dependencies(
  odrive_controller 
  PUBLIC
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  control_toolbox
  control_msgs
  realtime_tools
)
pluginlib_export_plugin_description_file(controller_interface odrive_controller.xml)
install(TARGETS odrive_controller odrive_controller_parameters EXPORT export_odrive_controller)
install(TARGETS odrive_controller EXPORT export_odrive_controller)
install(DIRECTORY controller/include DESTINATION share/odrive_controller)

######### ODrive Controller Value Publisher
add_executable(odrive_controller_value_publisher testing/odrive_controller_value_publisher.cpp)
ament_target_dependencies(odrive_controller_value_publisher rclcpp control_msgs)
install(TARGETS odrive_controller_value_publisher DESTINATION lib/${PROJECT_NAME})

######### ODrive Controller Sin Publisher
add_executable(odrive_controller_sin_publisher testing/odrive_controller_sin_publisher.cpp)
ament_target_dependencies(odrive_controller_sin_publisher rclcpp control_msgs)
install(TARGETS odrive_controller_sin_publisher DESTINATION lib/${PROJECT_NAME})

######### Install
# Add description files
install(
  DIRECTORY description/launch description/urdf
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
