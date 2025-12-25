#ifndef ODRIVE_ROS2_CONTROL_EXAMPLE__ROBOT_HARDWARE_INTERFACE_HPP_
#define ODRIVE_ROS2_CONTROL_EXAMPLE__ROBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace odrive_ros2_control_example
{
class RobotHardwareInterface : public hardware_interface::SystemInterface
{
  struct Joint {
    std::string name;
    int can_id;
    double reduction_ratio;
    double velocity_limit;
    double effort_limit;
    double position_command = 0.0;
    double velocity_command = 0.0;
    double effort_command = 0.0;
    double position_state = 0.0;
    double velocity_state = 0.0;
    double effort_state = 0.0;
    Joint(std::string name_, int can_id_, double reduction_ratio_, double velocity_limit_, double effort_limit_) 
      : name(name_), can_id(can_id_), reduction_ratio(reduction_ratio_), velocity_limit(velocity_limit_), effort_limit(effort_limit_) {}
  };

public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


private:
  int debug_;

  std::vector<Joint> joints;
};

}  // namespace odrive_ros2_control_example

#endif  // ODRIVE_ROS2_CONTROL_EXAMPLE__ROBOT_HARDWARE_INTERFACE_HPP_
