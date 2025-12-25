#include "odrive_ros2_control_example/robot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace odrive_ros2_control_example
{
  hardware_interface::CallbackReturn RobotHardwareInterface::on_init(
      const hardware_interface::HardwareComponentInterfaceParams & params)
  {
    if (
        hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    debug_ = stod(info_.hardware_parameters["debug"]);
    RCLCPP_INFO(get_logger(), "Debug: %f", debug_);
    hw_start_sec_ = 0.0;
    hw_stop_sec_ = 3.0;

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      RCLCPP_INFO(get_logger(), "Command Size: %zu", joint.command_interfaces.size());
      RCLCPP_INFO(get_logger(), "State Size: %zu", joint.state_interfaces.size());
      // // RRBotSystemPositionOnly has exactly one state and command interface on each joint
      // if (joint.command_interfaces.size() != 1)
      // {
      //   RCLCPP_FATAL(
      //       get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
      //       joint.name.c_str(), joint.command_interfaces.size());
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      // if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      // {
      //   RCLCPP_FATAL(
      //       get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
      //       joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
      //       hardware_interface::HW_IF_POSITION);
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      // if (joint.state_interfaces.size() != 1)
      // {
      //   RCLCPP_FATAL(
      //       get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
      //       joint.state_interfaces.size());
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      // {
      //   RCLCPP_FATAL(
      //       get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
      //       joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      //   return hardware_interface::CallbackReturn::ERROR;
      // }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RobotHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RobotHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    return hardware_interface::return_type::OK;
  }

}  // namespace odrive_ros2_control_example

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control_example::RobotHardwareInterface, hardware_interface::SystemInterface)
