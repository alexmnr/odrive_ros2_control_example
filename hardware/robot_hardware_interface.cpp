#include "odrive_ros2_control_example/robot_hardware_interface.hpp"

#include <cmath>
#include <string>
#include <vector>
#include "socket_can.hpp"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "odrive_enums.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace odrive_ros2_control_example
{
  ////////////////////// on_init /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    can_interface_name_ = info_.hardware_parameters["can_interface_name"];

    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "CAN inteface name: %s", can_interface_name_.c_str());

    for (const hardware_interface::ComponentInfo & joint_info : info_.joints)
    {
      // Check for correct size of command and state interfaces
      if (joint_info.command_interfaces.size() != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("RobotHardwareInterface"), "Joint '%s' has %zu command interfaces. 3 expected.", joint_info.name.c_str(), joint_info.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint_info.state_interfaces.size() != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("RobotHardwareInterface"), "Joint '%s' has %zu state interfaces. 3 expected.", joint_info.name.c_str(), joint_info.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // create joint object
      std::string name_ = joint_info.name;
      int can_id_ = (uint8_t)std::stoi(joint_info.parameters.at("can_id"));
      double reduction_ratio_ = std::stod(joint_info.parameters.at("reduction_ratio"));
      double velocity_limit_ = std::stod(joint_info.parameters.at("velocity_limit"));
      double effort_limit_ = std::stod(joint_info.parameters.at("effort_limit"));
      joints.push_back(Joint(name_, can_id_, reduction_ratio_, velocity_limit_, effort_limit_));
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Initialized Joint '%s' with can_id: %d reduction: %.1f velocity_limit: %.1f effort_limit: %.1f", name_.c_str(), can_id_, reduction_ratio_, velocity_limit_, effort_limit_);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    if (!can_intf_.init(can_interface_name_, &event_loop_, std::bind(&RobotHardwareInterface::on_can_msg, this, _1))) {
      RCLCPP_ERROR(
          rclcpp::get_logger("ODriveHardwareInterface"),
          "Failed to initialize SocketCAN on %s",
          can_interface_name_.c_str()
          );
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Initialized SocketCAN on %s", can_interface_name_.c_str());
    for (auto& joint : joints) {
      joint.can_intf = &can_intf_;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  ////////////////////// read /////////////////////////
  hardware_interface::return_type RobotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    while (can_intf_.read_nonblocking()) {
      // repeat until CAN interface has no more messages
    }
    // Request Torques and Encoder Estimates
    for (auto &joint : joints) {
      Get_Torques_msg_t get_torques_msg;
      Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
      joint.send(get_torques_msg, true);
      joint.send(get_encoder_estimages_msg, true);
    }

    return hardware_interface::return_type::OK;
  }

  ////////////////////// write /////////////////////////
  hardware_interface::return_type RobotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // for (auto &joint : joints) {
    // RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Write: Position: %f Velocity: %f Effort: %f", joint.position_command, joint.velocity_command, joint.effort_command);
    // }
    return hardware_interface::return_type::OK;
  }

  ////////////////////// on_deactivate /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_cleanup /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_shutdown /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// export_state_interfaces /////////////////////////
  std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto &joint : joints) {
      state_interfaces.emplace_back(joint.name, "position", &joint.position_state);
      state_interfaces.emplace_back(joint.name, "velocity", &joint.velocity_state);
      state_interfaces.emplace_back(joint.name, "effort", &joint.effort_state);
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

  void RobotHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& joint : joints) {
      if ((frame.can_id >> 5) == joint.can_id) {
        joint.on_can_msg(frame);
      }
    }
  }

  void RobotHardwareInterface::Joint::on_can_msg(const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
      if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
        RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "message %d too short", cmd);
        return false;
      }
      msg.decode_buf(frame.data);
      return true;
    };

    switch (cmd) {
      case Get_Encoder_Estimates_msg_t::cmd_id: {
        if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
          position_state = msg.Pos_Estimate * (2 * M_PI);
          velocity_state = msg.Vel_Estimate * (2 * M_PI);
        }
      } break;
      case Get_Torques_msg_t::cmd_id: {
        if (Get_Torques_msg_t msg; try_decode(msg)) {
          effort_target = msg.Torque_Target;
          effort_state = msg.Torque_Estimate;
        }
      } break;
    }
  }


}  // namespace odrive_ros2_control_example

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control_example::RobotHardwareInterface, hardware_interface::SystemInterface)
