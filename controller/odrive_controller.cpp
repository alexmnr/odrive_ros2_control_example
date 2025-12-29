#include "odrive_controller/odrive_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace odrive_controller
{
ODriveController::ODriveController() : controller_interface::ControllerInterface() {}

////////////////////// on_init /////////////////////////
controller_interface::CallbackReturn ODriveController::on_init() {
  try {
    param_listener = std::make_shared<ParamListener>(get_node());
    params = param_listener->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveController"), "[INIT] Exception thrown when reading parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////// on_configure /////////////////////////
controller_interface::CallbackReturn ODriveController::on_configure(const rclcpp_lifecycle::State &) {
  for (size_t i = 0; i < params.joints.size(); ++i)
  {
    ODriveController::Joint joint;
    joint.name = params.joints[i];
    joint.id = i;
    control_toolbox::AntiWindupStrategy anti_windup_strategy_;
    anti_windup_strategy_.set_type("none");
    try {
      anti_windup_strategy_.validate();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveController"), "[INIT] Invalid antiwindup strategy:: %s", e.what());
      return CallbackReturn::ERROR;
    }
    joint.pid = std::make_shared<control_toolbox::Pid>(
        params.gains.joints_map[joint.name].position_p,
        params.gains.joints_map[joint.name].position_i,
        params.gains.joints_map[joint.name].position_d,
        params.gains.joints_map[joint.name].position_output_max,
        params.gains.joints_map[joint.name].position_output_min,
        anti_windup_strategy_
        );
    joints.push_back(joint);
    RCLCPP_INFO(rclcpp::get_logger("ODriveController"), "[PARAM] Name: %s", params.joints[i].c_str());
  }

  joints_cmd_sub = this->get_node()->create_subscription<CommandType>("odrive_controller/command", rclcpp::SystemDefaultsQoS(),
    [this](const CommandType::SharedPtr msg) {
      rt_buffer.set(*msg);
    });

  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ODriveController::on_activate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ODriveController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  // get current states
  for (auto& joint : joints) {
    const auto position_op = state_interfaces_[joint.id].get_optional();
    if (position_op.has_value()) {joint.position_state = position_op.value();}
    const auto velocity_op = state_interfaces_[joint.id + 1].get_optional();
    if (velocity_op.has_value()) {joint.velocity_state = velocity_op.value();}
    const auto effort_op = state_interfaces_[joint.id + 2].get_optional();
    if (effort_op.has_value()) {joint.effort_state = effort_op.value();}
    RCLCPP_INFO(rclcpp::get_logger("ODriveController"), "Joint '%s' State: pos: %f vel: %f effort: %f Command: pos: %f", joint.name.c_str(), joint.position_state, joint.velocity_state, joint.effort_state, joint.position_command);
  }

  // get current command
  auto command_op = rt_buffer.try_get();
  if (command_op.has_value())
  {
    command_msg = command_op.value();
    int i = 0;
    for (std::string joint_name : command_msg.joint_names) {
      auto it = std::find_if(joints.begin(), joints.end(), [&](const Joint& joint) {
          return joint.name == joint_name;
          });

      // Check if the element was found
      if (it != joints.end()) {
        // checks
        if (command_msg.interface_values[i].interface_names.size() != 1) {
          RCLCPP_ERROR(rclcpp::get_logger("ODriveController"), "Wrong number of command interfaces! Expected 1 Got %zu", command_msg.interface_values.size());
          continue;
        }
        if (command_msg.interface_values[i].interface_names[0] != "position") {
          RCLCPP_ERROR(rclcpp::get_logger("ODriveController"), "Wrong type of command interface! Expected 'position' Got '%s'", command_msg.interface_values[i].interface_names[0].c_str());
          continue;
        }
        // save command
        it->position_command = command_msg.interface_values[i].values[0];
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveController"), "Joint of name '%s' not recognized!", joint_name.c_str());
      }
      i++;
    }
  }

  // write command state
  for (auto& joint : joints) {
    if (!std::isnan(joint.position_command)) {
      if (!command_interfaces_[joint.id].set_value(joint.position_command)) {
        RCLCPP_WARN(rclcpp::get_logger("ODriveController"), "Failed to set command interface value for joint '%s'", joint.name.c_str());
      }
    } 
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ODriveController::on_deactivate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ODriveController::command_interface_configuration() const {

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::vector<std::string> names_;
  for (const auto& joint : joints) {
    names_.push_back(joint.name + "/position");
  }
  command_interfaces_config.names = names_;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ODriveController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::vector<std::string> names_;
  for (const auto& joint : joints) {
    names_.push_back(joint.name + "/position");
    names_.push_back(joint.name + "/velocity");
    names_.push_back(joint.name + "/effort");
  }
  state_interfaces_config.names = names_;
  return state_interfaces_config;
}

}  // namespace odrive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_controller::ODriveController, controller_interface::ControllerInterface)
