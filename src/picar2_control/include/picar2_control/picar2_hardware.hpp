#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace picar2_control
{

class Picar2Hardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string port_;
  int baud_{460800};
  int fd_{-1};

  // State — rear drive wheels (encoder ticks → rad, rad/s)
  double pos_back_left_{0.0};
  double pos_back_right_{0.0};
  double vel_back_left_{0.0};
  double vel_back_right_{0.0};

  // State — front steer (rad); right mirrors left
  double pos_steer_left_{0.0};
  double pos_steer_right_{0.0};

  // Command — rear wheel velocity (rad/s), front steer position (rad)
  double cmd_vel_back_left_{0.0};
  double cmd_vel_back_right_{0.0};
  double cmd_steer_{0.0};
};

}  // namespace picar2_control
