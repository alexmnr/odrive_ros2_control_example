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
  ////////////////////// on_init /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
    RCLCPP_INFO(get_logger(), "Initializing ...please wait...");
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    debug_ = (int)stod(info_.hardware_parameters["debug"]);
    RCLCPP_INFO(get_logger(), "Debug: %d", debug_);

    for (const hardware_interface::ComponentInfo & joint_info : info_.joints)
    {
      // Check for correct size of command and state interfaces
      if (joint_info.command_interfaces.size() != 3) {
        RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces. 3 expected.", joint_info.name.c_str(), joint_info.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint_info.state_interfaces.size() != 3) {
        RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interfaces. 3 expected.", joint_info.name.c_str(), joint_info.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // create joint object
      std::string name_ = joint_info.name;
      int can_id_ = std::stoi(joint_info.parameters.at("can_id"));
      double reduction_ratio_ = std::stod(joint_info.parameters.at("reduction_ratio"));
      double velocity_limit_ = std::stod(joint_info.parameters.at("velocity_limit"));
      double effort_limit_ = std::stod(joint_info.parameters.at("effort_limit"));
      joints.push_back(Joint(name_, can_id_, reduction_ratio_, velocity_limit_, effort_limit_));
    }

    RCLCPP_INFO(get_logger(), "Successfully initialized!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully configure!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  ////////////////////// read /////////////////////////
  hardware_interface::return_type RobotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    // for (auto &joint : joints) {
      // joint.position_state = joint.position_command;
      // joint.velocity_state = joint.velocity_command;
      // joint.effort_state = joint.effort_command;
      // RCLCPP_INFO(get_logger(), "Read: Position: %f Velocity: %f Effort: %f", joint.position_state, joint.velocity_state, joint.effort_state);
    // }
    return hardware_interface::return_type::OK;
  }

  ////////////////////// write /////////////////////////
  hardware_interface::return_type RobotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // for (auto &joint : joints) {
      // RCLCPP_INFO(get_logger(), "Write: Position: %f Velocity: %f Effort: %f", joint.position_command, joint.velocity_command, joint.effort_command);
    // }
    return hardware_interface::return_type::OK;
  }

  ////////////////////// on_deactivate /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  ////////////////////// on_cleanup /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_shutdown /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Shutting down ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully shut down!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// export_state_interfaces /////////////////////////
  std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces() {
      std::vector<hardware_interface::StateInterface> state_interfaces;
      for (auto &joint : joints) {
        state_interfaces.emplace_back(joint.name, "position", &joint.position_command);
        state_interfaces.emplace_back(joint.name, "velocity", &joint.velocity_command);
        state_interfaces.emplace_back(joint.name, "effort", &joint.effort_command);
      }
      return state_interfaces;
    }

  ////////////////////// export_command_interfaces /////////////////////////
  std::vector<hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces() {
      std::vector<hardware_interface::CommandInterface> command_interfaces;
      for (auto &joint : joints) {
        command_interfaces.emplace_back(joint.name, "position", &joint.position_command);
        command_interfaces.emplace_back(joint.name, "velocity", &joint.velocity_command);
        command_interfaces.emplace_back(joint.name, "effort", &joint.effort_command);
      }
      return command_interfaces;
    }


}  // namespace odrive_ros2_control_example

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control_example::RobotHardwareInterface, hardware_interface::SystemInterface)
