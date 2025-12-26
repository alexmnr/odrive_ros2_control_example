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

    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[INIT] CAN inteface name: %s", can_interface_name_.c_str());

    for (const hardware_interface::ComponentInfo & joint_info : info_.joints)
    {
      // Check for correct size of command and state interfaces
      if (joint_info.command_interfaces.size() != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("RobotHardwareInterface"), "[INIT] Joint '%s' has %zu command interfaces. 3 expected.", joint_info.name.c_str(), joint_info.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint_info.state_interfaces.size() != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("RobotHardwareInterface"), "[INIT] Joint '%s' has %zu state interfaces. 3 expected.", joint_info.name.c_str(), joint_info.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // create joint object
      std::string name_ = joint_info.name;
      int can_id_ = (uint8_t)std::stoi(joint_info.parameters.at("can_id"));
      double reduction_ratio_ = std::stod(joint_info.parameters.at("reduction_ratio"));
      double velocity_limit_ = std::stod(joint_info.parameters.at("velocity_limit"));
      double effort_limit_ = std::stod(joint_info.parameters.at("effort_limit"));
      joints.push_back(Joint(name_, can_id_, reduction_ratio_, velocity_limit_, effort_limit_));
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[INIT] Initialized Joint '%s' with can_id: %d reduction: %.1f velocity_limit: %.1f effort_limit: %.1f", name_.c_str(), can_id_, reduction_ratio_, velocity_limit_, effort_limit_);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    if (!can_intf_.init(can_interface_name_, &event_loop_, std::bind(&RobotHardwareInterface::on_can_msg, this, _1))) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Failed to initialize SocketCAN on %s", can_interface_name_.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Have you run 'sudo ip link set up can0 type can bitrate 1000000'?");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Initialized SocketCAN on %s", can_interface_name_.c_str());
    for (auto& joint : joints) {
      joint.can_intf = &can_intf_;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    // broadcast clear_error msg
    clear_all_errors();

    // Configure all axis
    for (auto& joint : joints) {
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      joint.send(set_axis_state_msg);
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_POS_FILTER;
      joint.send(set_controller_mode_msg);
    }

    // Check if all joints are ready
    while (1) {
      while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
      }
      bool finish = true;
      for (auto &joint : joints) {
        if (joint.odrive_error == 0) {
          joint.ready = true;
          RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[ACTIVATION] Motor with can_id %d for joint '%s' is ready!", joint.can_id, joint.name.c_str());
        } else {
          // keep interface from activating if any motor has an error
          finish = false;
        }
      }
      if (finish) {
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[ACTIVATION] Hardware succesfully activated!");
        break;
        // return hardware_interface::CallbackReturn::SUCCESS;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[ACTIVATION] Hardware not ready yet...");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }


  ////////////////////// read /////////////////////////
  hardware_interface::return_type RobotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    while (can_intf_.read_nonblocking()) {
      // repeat until CAN interface has no more messages
    }

    ////// CONTINUE ONLY IF READY
    for (auto &joint : joints) {
      if (!joint.ready) {
        return hardware_interface::return_type::OK;
      }
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
    for (auto &joint : joints) {
      Set_Input_Pos_msg_t msg;
      msg.Input_Pos = joint.position_command / (2 * M_PI) * joint.reduction_ratio;
      msg.Vel_FF = 0.0;
      msg.Torque_FF = 0.0;
      // msg.Vel_FF = axis.vel_input_enabled_ ? (axis.vel_setpoint_ / (2 * M_PI)) : 0.0f;
      // msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
      joint.send(msg);
    }
    return hardware_interface::return_type::OK;
  }

  ////////////////////// on_deactivate /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    // Configure all axis
    for (auto& joint : joints) {
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_IDLE;
      joint.send(set_axis_state_msg);
    }
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

  ////////////////////// handle messages /////////////////////////
  void RobotHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& joint : joints) {
      if ((frame.can_id >> 5) == joint.can_id) {
        joint.on_can_msg(frame);
      }
    }
  }

  void RobotHardwareInterface::Joint::on_can_msg(const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    switch (cmd) {
      case Get_Encoder_Estimates_msg_t::cmd_id: {
        on_encoder_feedback(frame);
      } break;
      case Get_Torques_msg_t::cmd_id: {
        on_torque_feedback(frame);
      } break;
      case Heartbeat_msg_t::cmd_id: {
        on_heartbeat(frame);
      } break;
    }
  }

  ////////////////////// joint functions /////////////////////////
  void RobotHardwareInterface::Joint::on_encoder_feedback(const can_frame& frame) {
    if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "message %d too short", Get_Encoder_Estimates_msg_t::cmd_id);
      return;
    }
    Get_Encoder_Estimates_msg_t msg;
    msg.decode_buf(frame.data);
    position_state = msg.Pos_Estimate * (2 * M_PI) / reduction_ratio;
    velocity_state = msg.Vel_Estimate * (2 * M_PI) / reduction_ratio;
  }
  void RobotHardwareInterface::Joint::on_torque_feedback(const can_frame& frame) {
    if (frame.can_dlc < Get_Torques_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "message %d too short", Get_Torques_msg_t::cmd_id);
      return;
    }
    Get_Torques_msg_t msg;
    msg.decode_buf(frame.data);
    effort_target = msg.Torque_Target * reduction_ratio;
    effort_state = msg.Torque_Estimate * reduction_ratio;
  }
  void RobotHardwareInterface::Joint::on_heartbeat(const can_frame& frame) {
    if (frame.can_dlc < Heartbeat_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "message %d too short", Heartbeat_msg_t::cmd_id);
      return;
    }
    Heartbeat_msg_t msg;
    msg.decode_buf(frame.data);
    odrive_error = msg.Axis_Error;
    odrive_state = msg.Axis_State;
    odrive_procedure_result = msg.Procedure_Result;
    odrive_trajectory_done = msg.Trajectory_Done_Flag;
    if (odrive_error != 0) {
      ready = false;
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Joint '%s' with can-id '%d' has error '%d' and state '%d'", name.c_str(), can_id, odrive_error, odrive_state);
    }
  }
  void RobotHardwareInterface::Joint::clear_error() {
    Clear_Errors_msg_t msg;
    msg.Identify = 0;
    send(msg);
  }

  ////////////////////// helper functions /////////////////////////
  void RobotHardwareInterface::clear_all_errors() {
    for (auto& joint : joints) {
      joint.clear_error();
    }
  }


}  // namespace odrive_ros2_control_example

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control_example::RobotHardwareInterface, hardware_interface::SystemInterface)
