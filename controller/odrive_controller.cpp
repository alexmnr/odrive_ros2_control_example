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

controller_interface::CallbackReturn ODriveController::on_init() {
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ODriveController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
  return conf;
}

controller_interface::InterfaceConfiguration ODriveController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
  return conf;
}

controller_interface::CallbackReturn ODriveController::on_configure(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ODriveController::on_activate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ODriveController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ODriveController::on_deactivate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

}  // namespace odrive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_controller::ODriveController, controller_interface::ControllerInterface)
