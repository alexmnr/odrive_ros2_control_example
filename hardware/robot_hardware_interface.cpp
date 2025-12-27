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

    // Load hardware parameters
    can_interface_name_ = info_.hardware_parameters["can_interface_name"];
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] CAN inteface: %s", can_interface_name_.c_str());
    control_mode_ = info_.hardware_parameters["control_mode"];
    if (info_.hardware_parameters["control_mode"] == "position_filtered") {
      odrive_control_mode_ = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      odrive_input_mode_ = ODriveInputMode::INPUT_MODE_POS_FILTER;
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] Control Mode: Filtered Position Control");
    } else if (info_.hardware_parameters["control_mode"] == "position_trajectory") {
      odrive_control_mode_ = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      odrive_input_mode_ = ODriveInputMode::INPUT_MODE_TRAP_TRAJ;
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] Control Mode: Trajectory Control");
    } else if (control_mode_ == "torque") {
      odrive_control_mode_ = ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL;
      odrive_input_mode_ = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] Control Mode: Torque Control");
    } else {
      RCLCPP_FATAL(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] Unrecognized control mode: '%s'", control_mode_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // create joint objects 
    for (const hardware_interface::ComponentInfo & joint_info : info_.joints)
    {
      // Check for correct size of command and state interfaces
      if (joint_info.command_interfaces.size() != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] Joint '%s' has %zu command interfaces. 3 expected.", joint_info.name.c_str(), joint_info.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint_info.state_interfaces.size() != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] Joint '%s' has %zu state interfaces. 3 expected.", joint_info.name.c_str(), joint_info.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // create joint object
      RobotHardwareInterface::Joint joint;
      joint.name = joint_info.name;
      joint.can_id = (uint8_t)std::stoi(joint_info.parameters.at("can_id"));
      joint.joint_reduction_ratio = std::stod(joint_info.parameters.at("joint_reduction_ratio"));
      joint.motor_velocity_limit = std::stod(joint_info.parameters.at("motor_velocity_limit"));
      joint.motor_current_limit = std::stod(joint_info.parameters.at("motor_current_limit"));
      joint.position_p_gain = std::stod(joint_info.parameters.at("position_p_gain"));
      joint.velocity_p_gain = std::stod(joint_info.parameters.at("velocity_p_gain"));
      joint.velocity_i_gain = std::stod(joint_info.parameters.at("velocity_i_gain"));
      joint.input_filter_bandwith = std::stod(joint_info.parameters.at("input_filter_bandwith"));
      joint.trajectory_vel_limit = std::stod(joint_info.parameters.at("trajectory_vel_limit"));
      joint.trajectory_accel_limit = std::stod(joint_info.parameters.at("trajectory_accel_limit"));
      joint.trajectory_descel_limit = std::stod(joint_info.parameters.at("trajectory_descel_limit"));
      joint.trajectory_inertia = std::stod(joint_info.parameters.at("trajectory_inertia"));
      joints.push_back(joint);
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM] Created Joint '%s': ", joint.name.c_str());
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   can-id: %d", joint.can_id);
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   reduction: %.1f", joint.joint_reduction_ratio);
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   velocity-limit: %.1f", joint.motor_velocity_limit);
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   current-limit: %.1f", joint.motor_current_limit);
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   position-p-gain: %.3f", joint.position_p_gain);
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   velocity-p-gain: %.3f", joint.velocity_p_gain);
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   velocity-i-gain: %.3f", joint.velocity_i_gain);
      if (odrive_input_mode_ == ODriveInputMode::INPUT_MODE_POS_FILTER) {
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   input_filter_bandwith: %.1f", joint.input_filter_bandwith);
      } else if (odrive_input_mode_ == ODriveInputMode::INPUT_MODE_TRAP_TRAJ) {
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   trajectory_vel_limit: %.1f", joint.trajectory_vel_limit);
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   trajectory_accel_limit: %.1f", joint.trajectory_accel_limit);
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   trajectory_descel_limit: %.1f", joint.trajectory_descel_limit);
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[PARAM]   trajectory_inertia: %.1f", joint.trajectory_inertia);
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    // connect to CAN
    if (!can_intf_.init(can_interface_name_, &event_loop_, std::bind(&RobotHardwareInterface::on_can_msg, this, _1))) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Failed to initialize SocketCAN on %s", can_interface_name_.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Have you run 'sudo ip link set up can0 type can bitrate 1000000'?");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Succesfully initialized SocketCAN on %s", can_interface_name_.c_str());
    for (auto& joint : joints) {
      joint.can_intf = &can_intf_;
    }

    // Check if Motor connected and if version is correct
    for (auto& joint : joints) {
      Get_Version_msg_t msg;
      joint.send(msg, true);
    }
    for (int i = 0; i <= 30; i++) {
      bool finish = true;
      while (can_intf_.read_nonblocking()); // repeat until CAN interface has no more messages
      for (auto& joint : joints) {
        if (joint.hw_version.length() != 0 && joint.fw_version.length() != 0) {
          RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Found: Joint '%s' can-id '%d' with fw-version '%s' and hw-version '%s'", joint.name.c_str(), joint.can_id, joint.fw_version.c_str(), joint.hw_version.c_str());
          if (joint.fw_version != "0:6:11" || joint.hw_version != "5:2:0") {
            RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Joint '%s' with can-id '%d' is running the wrong version! Please update to 0:6:11 and 5:2:0.", joint.name.c_str(), joint.can_id);
            return CallbackReturn::FAILURE;
          }
        } else {
          finish = false;
        }
      }
      if (finish) {
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      if (i == 30) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Could not reach all motors! Check CAN connection!");
        return CallbackReturn::FAILURE;
      }
    }

    // write parameters
    for (auto& joint : joints) {
      joint.set_motor_limits();
      joint.set_trajectory_limits();
      joint.set_gains();
    }

    // clear all errors
    clear_all_errors();
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[CONFIG] Cleared all errors");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    // Arm all axis
    for (auto& joint : joints) {
      // closed loop control
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      joint.send(set_axis_state_msg);
      // control mode
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = odrive_control_mode_;
      set_controller_mode_msg.Input_Mode = odrive_input_mode_;
      joint.send(set_controller_mode_msg);
    }

    // Check if all joints are ready
    while (1) {
      // Request Torques and Encoder Estimates
      for (auto &joint : joints) {
        Get_Torques_msg_t get_torques_msg;
        Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
        joint.send(get_torques_msg, true);
        joint.send(get_encoder_estimages_msg, true);
      }
      while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
      }
      bool finish = true;
      for (auto &joint : joints) {
        if (joint.odrive_error == 0 && joint.position_state != 0) {
          joint.position_command = joint.position_state;
          joint.velocity_command = joint.velocity_command;
          joint.effort_command = joint.effort_command;
          joint.ready = true;
          RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[ACTIVATION] Motor with can-id '%d' for joint '%s' is ready!", joint.can_id, joint.name.c_str());
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

    // CONTINUE ONLY IF READY
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
    // CONTINUE ONLY IF READY
    for (auto &joint : joints) {
      if (!joint.ready) {
        return hardware_interface::return_type::OK;
      }
    }

    // set command position to all joints
    for (auto &joint : joints) {
      // position_filtered
      if (odrive_input_mode_ == ODriveInputMode::INPUT_MODE_POS_FILTER) {
        Set_Input_Pos_msg_t msg;
        msg.Input_Pos = joint.position_command / (2 * M_PI) * joint.joint_reduction_ratio;
        msg.Vel_FF = (joint.velocity_command / (2 * M_PI) * joint.joint_reduction_ratio);
        msg.Torque_FF = (joint.effort_command / (2 * M_PI) / joint.joint_reduction_ratio);
        // RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[DEBUG] Pos: %f Vel: %f Torque: %f", msg.Input_Pos, msg.Vel_FF, msg.Torque_FF);
        joint.send(msg);
      // position_trajectory
      } else if (odrive_input_mode_ == ODriveInputMode::INPUT_MODE_TRAP_TRAJ) {
        Set_Input_Pos_msg_t msg;
        msg.Input_Pos = joint.position_command / (2 * M_PI) * joint.joint_reduction_ratio;
        msg.Vel_FF = 0.0;
        msg.Torque_FF = 0.0;
        joint.send(msg);
      // torque_control
      } else if (odrive_input_mode_ == ODriveInputMode::INPUT_MODE_PASSTHROUGH) {
        Set_Input_Torque_msg_t msg;
        msg.Input_Torque = joint.effort_command;
        // RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "[DEBUG] Torque: %f Setpoint: %f", msg.Input_Torque, joint.effort_target);
        joint.send(msg);
      }
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
      case Get_Version_msg_t::cmd_id: {
        on_version_msg(frame);
      } break;
    }
  }

  ////////////////////// joint response functions /////////////////////////
  void RobotHardwareInterface::Joint::on_encoder_feedback(const can_frame& frame) {
    if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "message %d too short", Get_Encoder_Estimates_msg_t::cmd_id);
      return;
    }
    Get_Encoder_Estimates_msg_t msg;
    msg.decode_buf(frame.data);
    position_state = msg.Pos_Estimate * (2 * M_PI) / joint_reduction_ratio;
    velocity_state = msg.Vel_Estimate * (2 * M_PI) / joint_reduction_ratio;
  }
  void RobotHardwareInterface::Joint::on_torque_feedback(const can_frame& frame) {
    if (frame.can_dlc < Get_Torques_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"), "message %d too short", Get_Torques_msg_t::cmd_id);
      return;
    }
    Get_Torques_msg_t msg;
    msg.decode_buf(frame.data);
    effort_target = msg.Torque_Target * joint_reduction_ratio;
    effort_state = msg.Torque_Estimate * joint_reduction_ratio;
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
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "Joint '%s' with can-id '%d' has error '%d' and state '%d'", name.c_str(), can_id, odrive_error, odrive_state);
    }
  }
  void RobotHardwareInterface::Joint::on_version_msg(const can_frame& frame) {
    Get_Version_msg_t msg;
    msg.decode_buf(frame.data);
    hw_version = std::to_string(msg.Hw_Version_Major) + ":" + std::to_string(msg.Hw_Version_Minor) + ":" + std::to_string(msg.Hw_Version_Variant);
    fw_version = std::to_string(msg.Fw_Version_Major) + ":" + std::to_string(msg.Fw_Version_Minor) + ":" + std::to_string(msg.Fw_Version_Revision);
  }

  ////////////////////// joint helper functions /////////////////////////
  void RobotHardwareInterface::Joint::clear_error() {
    Clear_Errors_msg_t msg;
    msg.Identify = 0;
    send(msg);
  }
  void RobotHardwareInterface::Joint::request_encoder_feedback() {
    Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
    send(get_encoder_estimages_msg, true);
  }
  void RobotHardwareInterface::Joint::request_torques_feedback() {
    Get_Torques_msg_t get_torques_msg;
    send(get_torques_msg, true);
  }
  template <typename V>
  void RobotHardwareInterface::Joint::write_parameter(uint16_t endpoint_id, V value) {
    struct can_frame frame;
    frame.can_id = can_id << 5 | 0x04;
    frame.can_dlc = 8;
    can_set_signal_raw<uint8_t>(frame.data, 1, 0, 8, true);
    can_set_signal_raw<uint16_t>(frame.data, endpoint_id, 8, 24, true);
    can_set_signal_raw<uint8_t>(frame.data, 0, 24, 32, true);
    if constexpr (std::is_same_v<V, uint8_t>) {
      can_set_signal_raw<uint8_t>(frame.data, value, 32, 40, true);
    } else if constexpr (std::is_same_v<V, uint16_t>) {
      can_set_signal_raw<uint16_t>(frame.data, value, 32, 48, true);
    } else if constexpr (std::is_same_v<V, uint32_t>) {
      can_set_signal_raw<uint32_t>(frame.data, value, 32, 56, true);
    } else if constexpr (std::is_same_v<V, int8_t>) {
      can_set_signal_raw<int8_t>(frame.data, value, 32, 40, true);
    } else if constexpr (std::is_same_v<V, int16_t>) {
      can_set_signal_raw<int16_t>(frame.data, value, 32, 48, true);
    } else if constexpr (std::is_same_v<V, int32_t>) {
      can_set_signal_raw<int32_t>(frame.data, value, 32, 56, true);
    } else if constexpr (std::is_same_v<V, bool>) {
      can_set_signal_raw<bool>(frame.data, value, 32, 40, true);
    } else if constexpr (std::is_same_v<V, float>) {
      can_set_signal_raw<float>(frame.data, value, 32, 64, true);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "Unsupported type in write_parameter function");
      return;
    }
    can_intf->send_can_frame(frame);
  }
  void RobotHardwareInterface::Joint::set_motor_limits() {
    // Set Velocity Limit
    Set_Limits_msg_t msg;
    if (motor_velocity_limit != 0) {
      msg.Velocity_Limit = (float)motor_velocity_limit;
    } else {
      msg.Velocity_Limit = std::numeric_limits<float>::infinity();
    }
    // Set Soft Current Limit
    if (motor_current_limit != 0) {
      msg.Current_Limit = (float)motor_current_limit;
    } else {
      msg.Current_Limit = std::numeric_limits<float>::infinity();
    }
    send(msg);
  }
  void RobotHardwareInterface::Joint::set_trajectory_limits() {
    // Set Velocity Limit
    Set_Traj_Vel_Limit_msg_t vel_limit_msg;
    vel_limit_msg.Traj_Vel_Limit = (float)trajectory_vel_limit;
    send(vel_limit_msg);
    // Set Acceleration Limit
    Set_Traj_Accel_Limits_msg_t acc_limit_msg;
    acc_limit_msg.Traj_Accel_Limit = (float)trajectory_accel_limit;
    acc_limit_msg.Traj_Decel_Limit = (float)trajectory_descel_limit;
    send(acc_limit_msg);
    // Set Inertia
    Set_Traj_Inertia_msg_t inertia_msg;
    inertia_msg.Traj_Inertia = (float)trajectory_inertia;
    send(inertia_msg);
  }
  void RobotHardwareInterface::Joint::set_gains() {
    // Set Position Gains
    Set_Pos_Gain_msg_t pos_msg;
    pos_msg.Pos_Gain = (float)position_p_gain;
    send(pos_msg);
    // Set Velocity Gains
    Set_Vel_Gains_msg_t vel_msg;
    vel_msg.Vel_Gain = (float)velocity_p_gain;
    vel_msg.Vel_Integrator_Gain = (float)velocity_i_gain;
    send(vel_msg);
  }


  ////////////////////// general helper functions /////////////////////////
  void RobotHardwareInterface::clear_all_errors() {
    for (auto& joint : joints) {
      joint.clear_error();
    }
  }


}  // namespace odrive_ros2_control_example

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control_example::RobotHardwareInterface, hardware_interface::SystemInterface)
