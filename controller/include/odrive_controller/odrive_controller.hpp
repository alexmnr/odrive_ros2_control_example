#ifndef ODRIVE_CONTROLLER__ODRIVE_CONTROLLER_HPP_
#define ODRIVE_CONTROLLER__ODRIVE_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"

namespace odrive_controller
{
class ODriveController : public controller_interface::ControllerInterface
{
public:
  ODriveController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

  std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *> 
    command_interface_map_ = {{"effort", &joint_effort_command_interface_}};

  std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {{"position", &joint_position_state_interface_},{"velocity", &joint_velocity_state_interface_}};
};

}  // namespace odrive_controller

#endif  // ODRIVE_CONTROLLER__ODRIVE_CONTROLLER_HPP_
