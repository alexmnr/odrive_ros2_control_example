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
  # ros2_control_cmake
  # controller_manager
  # backward_ros
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
  hardware/robot_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE hardware/include)
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

# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
# target_include_directories(${PROJECT_NAME} PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
# $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# target_include_directories(${PROJECT_NAME} PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
# $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )
# target_link_libraries(${PROJECT_NAME} PUBLIC
#   hardware_interface::hardware_interface
#   pluginlib::pluginlib
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
# )
# ament_export_targets(export_odrive_ros2_control_example HAS_LIBRARY_TARGET)

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
