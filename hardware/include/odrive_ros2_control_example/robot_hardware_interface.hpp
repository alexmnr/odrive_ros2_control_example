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
#include "socket_can.hpp"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "odrive_enums.h"

namespace odrive_ros2_control_example
{
class RobotHardwareInterface : public hardware_interface::SystemInterface
{
  struct Joint {
    // parameters
    const std::string name;
    const uint8_t can_id;
    const double reduction_ratio;
    const double velocity_limit;
    const double effort_limit;
    // command variables
    double position_command = 0.0;
    double velocity_command = 0.0;
    double effort_command = 0.0;
    // state variables
    double position_state = 0.0;
    double velocity_state = 0.0;
    double effort_state = 0.0;
    double effort_target = 0.0;
    // other variables
    uint32_t odrive_error = 1;
    uint8_t odrive_state = 0;
    uint8_t odrive_procedure_result = 0;
    uint8_t odrive_trajectory_done = 0;
    bool ready = false;
    // Constructor Function
    Joint(std::string name_, uint8_t can_id_, double reduction_ratio_, double velocity_limit_, double effort_limit_) 
      : name(name_), can_id(can_id_), reduction_ratio(reduction_ratio_), velocity_limit(velocity_limit_), effort_limit(effort_limit_) {}
    // CAN variables
    SocketCanIntf* can_intf;
    // functions
    void on_can_msg(const can_frame& frame);
    void on_heartbeat(const can_frame& frame);
    void on_encoder_feedback(const can_frame& frame);
    void on_torque_feedback(const can_frame& frame);
    template <typename T>
    void send(const T& msg, bool rtr = false) const {
        struct can_frame frame;
        frame.can_id = can_id << 5 | msg.cmd_id;
        if (rtr) {
          frame.can_id |= CAN_RTR_FLAG;
          frame.can_dlc = 0;
        } else {
          frame.can_dlc = msg.msg_length;
          msg.encode_buf(frame.data);
        }
        can_intf->send_can_frame(frame);
    }
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
  std::string can_interface_name_;
  SocketCanIntf can_intf_;
  EpollEventLoop event_loop_;

  std::vector<Joint> joints;

  void on_can_msg(const can_frame& frame);
};

}  // namespace odrive_ros2_control_example

#endif  // ODRIVE_ROS2_CONTROL_EXAMPLE__ROBOT_HARDWARE_INTERFACE_HPP_
