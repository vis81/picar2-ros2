#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
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
  void reader_loop();
  void timesync_loop();
  void uart_write(const uint8_t * buf, int len);
  void process_byte(uint8_t b);
  void dispatch_joint_frame(const uint8_t * payload, uint8_t len);
  void dispatch_timesync_resp();

  std::string port_;
  int baud_{460800};
  int fd_{-1};

  // Joint state — exported via state interfaces; written by read() (main thread only)
  double pos_back_left_{0.0};
  double pos_back_right_{0.0};
  double vel_back_left_{0.0};
  double vel_back_right_{0.0};
  double pos_steer_left_{0.0};
  double pos_steer_right_{0.0};
  double pos_front_left_wheel_{0.0};
  double pos_front_right_wheel_{0.0};

  // Commands — written by write() (main thread only)
  double cmd_vel_back_left_{0.0};
  double cmd_vel_back_right_{0.0};
  double cmd_steer_left_{0.0};
  double cmd_steer_right_{0.0};

  // Frame decoder state — reader thread only, no locking needed
  enum class DecodeState : uint8_t { START, TYPE, LEN, PAYLOAD, CRC };
  DecodeState decode_state_{DecodeState::START};
  uint8_t rx_type_{0};
  uint8_t rx_len_{0};
  uint8_t rx_pos_{0};
  uint8_t rx_buf_[32]{};

  // Background thread lifecycle (both share reader_running_)
  std::thread       reader_thread_;
  std::thread       timesync_thread_;
  std::atomic<bool> reader_running_{false};
  std::mutex        write_mutex_;  // serialises all ::write(fd_) calls across threads

  // Shared state — protected by state_mutex_
  std::mutex              state_mutex_;
  std::condition_variable joint_cv_;
  bool    joint_ready_{false};

  // Staging area — reader thread writes, read() copies to exported state
  double  stg_pos_left_{0.0};
  double  stg_pos_right_{0.0};
  double  stg_vel_left_{0.0};
  double  stg_vel_right_{0.0};
  double  stg_steer_{0.0};
  int64_t stg_pi_time_us_{0};  // 0 while timesync not yet valid
  int64_t stg_t3_us_{0};       // CLOCK_MONOTONIC µs when JOINT frame arrived

  // Timesync — sync_last_t4_us_ written by reader thread, read by read()
  int64_t sync_last_t4_us_{0};
};

}  // namespace picar2_control
