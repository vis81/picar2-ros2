#include "picar2_control/picar2_hardware.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace picar2_control
{

// ── Protocol constants ────────────────────────────────────────────────────────
static constexpr uint8_t PROTO_START   = 0xAA;
static constexpr uint8_t PROTO_MAX_LEN = 32;

static constexpr uint8_t MSG_CMD_VEL       = 0x80;
static constexpr uint8_t MSG_SET_RATE      = 0x82;
static constexpr uint8_t MSG_TIMESYNC      = 0x84;
static constexpr uint8_t STREAM_JOINT      = 0x01;
static constexpr uint8_t MSG_TIMESYNC_RESP = 0x05;

static constexpr double DEG_TO_RAD    = M_PI / 180.0;
static constexpr double RAD_TO_DEG    = 180.0 / M_PI;
static constexpr double STEER_MAX_RAD = 0.6;  // URDF steer limit

// ── CRC-8 (poly 0x31, Dallas/Maxim) — matches firmware protocol.c ────────────
static uint8_t crc8(const uint8_t * buf, size_t len)
{
  uint8_t crc = 0;
  while (len--) {
    crc ^= *buf++;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
    }
  }
  return crc;
}

// ── Frame encoder — matches firmware proto_encode() ───────────────────────────
static int encode_frame(uint8_t type, const uint8_t * payload, uint8_t len, uint8_t * out)
{
  out[0] = PROTO_START;
  out[1] = type;
  out[2] = len;
  if (len > 0) {
    std::memcpy(&out[3], payload, len);
  }
  out[3 + len] = crc8(&out[1], 2 + len);
  return 4 + len;
}

// ── Little-endian helpers ─────────────────────────────────────────────────────
static inline int16_t get_le16(const uint8_t * p)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

static inline int32_t get_le32(const uint8_t * p)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(p[0]) |
    (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) |
    (static_cast<uint32_t>(p[3]) << 24));
}

static inline int64_t get_le64(const uint8_t * p)
{
  return static_cast<int64_t>(
    static_cast<uint64_t>(get_le32(p)) | (static_cast<uint64_t>(get_le32(p + 4)) << 32));
}

static inline void put_le32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}

static inline void put_le64(uint8_t * p, int64_t v)
{
  put_le32(p,     static_cast<uint32_t>(static_cast<uint64_t>(v) & 0xFFFFFFFFu));
  put_le32(p + 4, static_cast<uint32_t>(static_cast<uint64_t>(v) >> 32));
}

static inline void put_le16(uint8_t * p, int16_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

static int64_t time_now_us()
{
  return rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds() / 1000LL;
}

// ── Serial helpers ────────────────────────────────────────────────────────────
static speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return B460800;
  }
}

// ── on_init ───────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn Picar2Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_ = info_.hardware_parameters.count("port")
    ? info_.hardware_parameters.at("port") : "/dev/ttyYahboom0";
  baud_ = info_.hardware_parameters.count("baud")
    ? std::stoi(info_.hardware_parameters.at("baud")) : 460800;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure — open and configure serial port ────────────────────────────
hardware_interface::CallbackReturn Picar2Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Cannot open %s: %s", port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr: %s", std::strerror(errno));
    ::close(fd_); fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, baud_to_speed(baud_));
  cfsetospeed(&tty, baud_to_speed(baud_));
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr: %s", std::strerror(errno));
    ::close(fd_); fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }
  tcflush(fd_, TCIOFLUSH);

  RCLCPP_INFO(get_logger(), "Opened %s at %d baud", port_.c_str(), baud_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate — start JOINT stream, reset decoder ──────────────────────────
hardware_interface::CallbackReturn Picar2Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Request JOINT stream at 50 Hz
  uint8_t payload[3] = {STREAM_JOINT, 50, 0};
  uint8_t frame[16];
  int n = encode_frame(MSG_SET_RATE, payload, sizeof(payload), frame);
  ::write(fd_, frame, n);

  decode_state_    = DecodeState::START;
  last_joint_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  write_cycle_         = 0;
  sync_last_t4_us_     = 0;

  RCLCPP_INFO(get_logger(), "Activated — JOINT stream at 50 Hz");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate — stop stream, zero outputs, close port ────────────────────
hardware_interface::CallbackReturn Picar2Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint8_t frame[16];
  int n;

  // Stop JOINT stream (hz = 0)
  uint8_t stop_payload[3] = {STREAM_JOINT, 0, 0};
  n = encode_frame(MSG_SET_RATE, stop_payload, sizeof(stop_payload), frame);
  ::write(fd_, frame, n);

  // Zero velocity, neutral steer
  uint8_t vel_payload[5] = {0, 0, 0, 0, 50};
  n = encode_frame(MSG_CMD_VEL, vel_payload, sizeof(vel_payload), frame);
  ::write(fd_, frame, n);

  ::close(fd_);
  fd_ = -1;

  RCLCPP_INFO(get_logger(), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Joint interface exports ───────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface> Picar2Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back("back_left_joint",         hardware_interface::HW_IF_POSITION, &pos_back_left_);
  interfaces.emplace_back("back_left_joint",         hardware_interface::HW_IF_VELOCITY, &vel_back_left_);
  interfaces.emplace_back("back_right_joint",        hardware_interface::HW_IF_POSITION, &pos_back_right_);
  interfaces.emplace_back("back_right_joint",        hardware_interface::HW_IF_VELOCITY, &vel_back_right_);
  interfaces.emplace_back("front_left_steer_joint",  hardware_interface::HW_IF_POSITION, &pos_steer_left_);
  interfaces.emplace_back("front_right_steer_joint", hardware_interface::HW_IF_POSITION, &pos_steer_right_);
  interfaces.emplace_back("front_left_wheel_joint",  hardware_interface::HW_IF_POSITION, &pos_front_left_wheel_);
  interfaces.emplace_back("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &pos_front_right_wheel_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Picar2Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back("back_left_joint",         hardware_interface::HW_IF_VELOCITY, &cmd_vel_back_left_);
  interfaces.emplace_back("back_right_joint",        hardware_interface::HW_IF_VELOCITY, &cmd_vel_back_right_);
  interfaces.emplace_back("front_left_steer_joint",  hardware_interface::HW_IF_POSITION, &cmd_steer_left_);
  interfaces.emplace_back("front_right_steer_joint", hardware_interface::HW_IF_POSITION, &cmd_steer_right_);
  return interfaces;
}

// ── Frame decoder ─────────────────────────────────────────────────────────────
void Picar2Hardware::dispatch_joint_frame(const uint8_t * p, uint8_t len, const rclcpp::Time & t)
{
  double new_left  = get_le32(&p[0]) * DEG_TO_RAD;
  double new_right = get_le32(&p[4]) * DEG_TO_RAD;
  double new_steer = (50 - static_cast<int>(p[8])) / 50.0 * STEER_MAX_RAD;

  if (len >= 14) {
    vel_back_left_  = get_le16(&p[10]) * DEG_TO_RAD;
    vel_back_right_ = get_le16(&p[12]) * DEG_TO_RAD;
  } else if (last_joint_time_.nanoseconds() > 0) {
    double dt = (t - last_joint_time_).seconds();
    if (dt > 1e-3 && dt < 1.0) {
      vel_back_left_  = (new_left  - pos_back_left_)  / dt;
      vel_back_right_ = (new_right - pos_back_right_) / dt;
    }
  }

  if (len >= 22) {
    int64_t pi_us = get_le64(&p[14]);
    if (pi_us != 0) {
      last_corrected_stamp_ = rclcpp::Time(pi_us * 1000LL, RCL_ROS_TIME);
    }
  }

  pos_back_left_   = new_left;
  pos_back_right_  = new_right;
  pos_steer_left_  = new_steer;
  pos_steer_right_ = new_steer;
  last_joint_time_ = t;
}

void Picar2Hardware::process_byte(uint8_t b, const rclcpp::Time & t)
{
  switch (decode_state_) {
    case DecodeState::START:
      if (b == PROTO_START) {
        decode_state_ = DecodeState::TYPE;
      }
      break;

    case DecodeState::TYPE:
      rx_type_      = b;
      decode_state_ = DecodeState::LEN;
      break;

    case DecodeState::LEN:
      if (b > PROTO_MAX_LEN) {
        decode_state_ = DecodeState::START;
        break;
      }
      rx_len_       = b;
      rx_pos_       = 0;
      decode_state_ = (b > 0) ? DecodeState::PAYLOAD : DecodeState::CRC;
      break;

    case DecodeState::PAYLOAD:
      rx_buf_[rx_pos_++] = b;
      if (rx_pos_ == rx_len_) {
        decode_state_ = DecodeState::CRC;
      }
      break;

    case DecodeState::CRC: {
      uint8_t crc_input[2 + PROTO_MAX_LEN];
      crc_input[0] = rx_type_;
      crc_input[1] = rx_len_;
      std::memcpy(&crc_input[2], rx_buf_, rx_len_);

      if (b == crc8(crc_input, 2 + rx_len_)) {
        if (rx_type_ == STREAM_JOINT && rx_len_ >= 10) {
          dispatch_joint_frame(rx_buf_, rx_len_, t);
        } else if (rx_type_ == MSG_TIMESYNC_RESP && rx_len_ >= 8) {
          dispatch_timesync_resp();
        }
      }
      decode_state_ = DecodeState::START;
      break;
    }
  }
}

// ── read — drain serial, parse frames ────────────────────────────────────────
hardware_interface::return_type Picar2Hardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  uint8_t buf[256];
  ssize_t n = ::read(fd_, buf, sizeof(buf));
  for (ssize_t i = 0; i < n; i++) {
    process_byte(buf[i], time);
  }
  return hardware_interface::return_type::OK;
}

// ── write — encode and send CMD_VEL every cycle (feeds watchdog) ──────────────
hardware_interface::return_type Picar2Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  // rad/s → deg/s, clamped to int16
  auto to_dps = [](double rads) -> int16_t {
    double dps = rads * RAD_TO_DEG;
    return static_cast<int16_t>(
      std::clamp(dps, static_cast<double>(INT16_MIN), static_cast<double>(INT16_MAX)));
  };

  // rad → 0-100 (50 = center). Servo convention: 0 = full left, 100 = full right,
  // so positive rad (left) maps to val < 50.
  auto to_steer = [](double rad) -> uint8_t {
    double val = std::round(50.0 - rad / STEER_MAX_RAD * 50.0);
    return static_cast<uint8_t>(std::clamp(val, 0.0, 100.0));
  };

  uint8_t payload[5];
  // Average Ackermann inner/outer angles — our servo applies one physical angle
  double cmd_steer = (cmd_steer_left_ + cmd_steer_right_) * 0.5;

  put_le16(&payload[0], to_dps(cmd_vel_back_left_));
  put_le16(&payload[2], to_dps(cmd_vel_back_right_));
  payload[4] = to_steer(cmd_steer);

  uint8_t frame[16];
  int n = encode_frame(MSG_CMD_VEL, payload, sizeof(payload), frame);
  ::write(fd_, frame, n);

  if (write_cycle_ < 8 || write_cycle_ % 50 == 0) {
    send_timesync();
  }
  write_cycle_++;

  return hardware_interface::return_type::OK;
}

void Picar2Hardware::send_timesync()
{
  sync_t1_us_ = time_now_us();
  uint8_t payload[16];
  put_le64(&payload[0], sync_t1_us_);
  put_le64(&payload[8], sync_last_t4_us_);
  uint8_t frame[24];
  int n = encode_frame(MSG_TIMESYNC, payload, 16, frame);
  ::write(fd_, frame, n);
}

void Picar2Hardware::dispatch_timesync_resp()
{
  sync_last_t4_us_ = time_now_us();
  RCLCPP_DEBUG(get_logger(), "sync: T4 recorded");
}

}  // namespace picar2_control

PLUGINLIB_EXPORT_CLASS(picar2_control::Picar2Hardware, hardware_interface::SystemInterface)
