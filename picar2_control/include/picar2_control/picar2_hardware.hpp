#pragma once

#include <cstdint>
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
  void process_byte(uint8_t b, const rclcpp::Time & t);
  void dispatch_joint_frame(const uint8_t * payload, const rclcpp::Time & t);

  std::string port_;
  int baud_{460800};
  int fd_{-1};

  // Joint state — rear drive wheels
  double pos_back_left_{0.0};
  double pos_back_right_{0.0};
  double vel_back_left_{0.0};
  double vel_back_right_{0.0};

  // Joint state — front steer (rad); right mirrors left
  double pos_steer_left_{0.0};
  double pos_steer_right_{0.0};

  // Commands — rear wheel velocity (rad/s), front steer position (rad)
  double cmd_vel_back_left_{0.0};
  double cmd_vel_back_right_{0.0};
  double cmd_steer_left_{0.0};
  double cmd_steer_right_{0.0};

  // Frame decoder state machine
  enum class DecodeState : uint8_t { START, TYPE, LEN, PAYLOAD, CRC };
  DecodeState decode_state_{DecodeState::START};
  uint8_t rx_type_{0};
  uint8_t rx_len_{0};
  uint8_t rx_pos_{0};
  uint8_t rx_buf_[32]{};

  // Timestamp of last received JOINT frame (for velocity estimation)
  rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace picar2_control
