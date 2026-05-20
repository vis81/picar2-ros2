#include "picar2_control/picar2_hardware.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace picar2_control
{

// ── Protocol constants ────────────────────────────────────────────────────────
static constexpr uint8_t PROTO_START       = 0xAA;
static constexpr uint8_t PROTO_MAX_LEN     = 32;

static constexpr uint8_t MSG_CMD_VEL       = 0x80;
static constexpr uint8_t MSG_REQ           = 0x81;
static constexpr uint8_t MSG_SET_RATE      = 0x82;
static constexpr uint8_t MSG_TIMESYNC      = 0x84;
static constexpr uint8_t STREAM_JOINT      = 0x01;
static constexpr uint8_t STREAM_IMU        = 0x02;
static constexpr uint8_t MSG_TIMESYNC_RESP = 0x05;

static constexpr double DEG_TO_RAD    = M_PI / 180.0;
static constexpr double RAD_TO_DEG    = 180.0 / M_PI;
static constexpr double STEER_MAX_RAD = 0.6;

// ── CRC-8 (poly 0x31, Dallas/Maxim) ──────────────────────────────────────────
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

// ── Frame encoder ─────────────────────────────────────────────────────────────
static int encode_frame(uint8_t type, const uint8_t * payload, uint8_t len, uint8_t * out)
{
  out[0] = PROTO_START;
  out[1] = type;
  out[2] = len;
  if (len > 0) std::memcpy(&out[3], payload, len);
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
    static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

static inline int64_t get_le64(const uint8_t * p)
{
  uint64_t lo = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  uint64_t hi = static_cast<uint32_t>(p[4]) | (static_cast<uint32_t>(p[5]) << 8) |
                (static_cast<uint32_t>(p[6]) << 16) | (static_cast<uint32_t>(p[7]) << 24);
  return static_cast<int64_t>(lo | (hi << 32));
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

// ── CLOCK_MONOTONIC µs ────────────────────────────────────────────────────────
static inline int64_t now_us()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000LL +
         static_cast<int64_t>(ts.tv_nsec) / 1000LL;
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
  imu_rate_hz_ = info_.hardware_parameters.count("imu_rate_hz")
    ? std::stoi(info_.hardware_parameters.at("imu_rate_hz")) : 50;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure — open and configure serial port ─────────────────────────────
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

  imu_node_ = rclcpp::Node::make_shared("picar2_imu");
  imu_pub_  = imu_node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
  mag_pub_  = imu_node_->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate — start reader thread ────────────────────────────────────────
hardware_interface::CallbackReturn Picar2Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  decode_state_ = DecodeState::START;

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    joint_ready_     = false;
    stg_pi_time_us_  = 0;
    stg_t3_us_       = 0;
    sync_last_t4_us_ = 0;
  }

  reader_running_.store(true);
  reader_thread_   = std::thread(&Picar2Hardware::reader_loop,   this);
  timesync_thread_ = std::thread(&Picar2Hardware::timesync_loop, this);

  // Start IMU stream
  auto hz = static_cast<uint16_t>(imu_rate_hz_);
  uint8_t rate_payload[3] = {STREAM_IMU,
    static_cast<uint8_t>(hz & 0xFF), static_cast<uint8_t>(hz >> 8)};
  uint8_t rate_frame[10];
  uart_write(rate_frame, encode_frame(MSG_SET_RATE, rate_payload, 3, rate_frame));

  RCLCPP_INFO(get_logger(), "Activated — IMU streaming at %d Hz", imu_rate_hz_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate — stop reader thread, close port ───────────────────────────
hardware_interface::CallbackReturn Picar2Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reader_running_.store(false);
  joint_cv_.notify_all();
  if (reader_thread_.joinable())   reader_thread_.join();
  if (timesync_thread_.joinable()) timesync_thread_.join();

  // Stop IMU stream
  uint8_t stop_payload[3] = {STREAM_IMU, 0, 0};
  uint8_t stop_frame[10];
  uart_write(stop_frame, encode_frame(MSG_SET_RATE, stop_payload, 3, stop_frame));

  // Zero velocity, neutral steer (0 = center)
  uint8_t vel_payload[6] = {0, 0, 0, 0, 0, 0};
  uint8_t frame[16];
  uart_write(frame, encode_frame(MSG_CMD_VEL, vel_payload, sizeof(vel_payload), frame));

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

// ── UART write — mutex-guarded so reader_loop/timesync_loop/main don't race ───
void Picar2Hardware::uart_write(const uint8_t * buf, int len)
{
  std::lock_guard<std::mutex> lk(write_mutex_);
  int sent = 0;
  while (sent < len) {
    ssize_t n = ::write(fd_, buf + sent, len - sent);
    if (n > 0) {
      sent += n;
    } else if (errno != EAGAIN && errno != EINTR) {
      RCLCPP_ERROR(get_logger(), "uart_write: %s", std::strerror(errno));
      break;
    }
  }
}

// ── Reader thread — continuous UART drain ─────────────────────────────────────
void Picar2Hardware::reader_loop()
{
  while (reader_running_.load(std::memory_order_relaxed)) {
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd_, &rset);
    struct timeval tv{0, 10000};  // 10 ms wake interval
    if (select(fd_ + 1, &rset, nullptr, nullptr, &tv) <= 0) continue;

    uint8_t buf[64];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    for (ssize_t i = 0; i < n; i++) {
      process_byte(buf[i]);
    }
  }
}

// ── Timesync thread — periodic MSG_TIMESYNC, independent of read() ───────────
void Picar2Hardware::timesync_loop()
{
  while (reader_running_.load(std::memory_order_relaxed)) {
    int64_t t4_prev;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      t4_prev = sync_last_t4_us_;
    }

    int64_t t1 = now_us();
    uint8_t payload[16];
    put_le64(&payload[0], t1);
    put_le64(&payload[8], t4_prev);
    uint8_t frame[24];
    uart_write(frame, encode_frame(MSG_TIMESYNC, payload, 16, frame));

    // Sleep 1 s in 10 ms steps so we respond to shutdown quickly
    for (int i = 0; i < 100 && reader_running_.load(std::memory_order_relaxed); i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

// ── Frame decoder ─────────────────────────────────────────────────────────────
void Picar2Hardware::dispatch_joint_frame(const uint8_t * p, uint8_t len)
{
  int64_t t3 = now_us();

  double pos_left  = get_le32(&p[0]) * DEG_TO_RAD;
  double pos_right = get_le32(&p[4]) * DEG_TO_RAD;
  double steer     = get_le16(&p[8]) / 10.0 * DEG_TO_RAD;
  double vel_left  = get_le16(&p[11]) * DEG_TO_RAD;
  double vel_right = get_le16(&p[13]) * DEG_TO_RAD;
  int64_t pi_us    = (len >= 23) ? get_le64(&p[15]) : 0LL;

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    stg_pos_left_   = pos_left;
    stg_pos_right_  = pos_right;
    stg_vel_left_   = vel_left;
    stg_vel_right_  = vel_right;
    stg_steer_      = steer;
    stg_pi_time_us_ = pi_us;
    stg_t3_us_      = t3;
    joint_ready_    = true;
  }
  joint_cv_.notify_one();
}

void Picar2Hardware::dispatch_timesync_resp()
{
  int64_t t4 = now_us();
  std::lock_guard<std::mutex> lk(state_mutex_);
  sync_last_t4_us_ = t4;
}

void Picar2Hardware::dispatch_imu_frame(const uint8_t * p, uint8_t len)
{
  if (len < 12) return;  // need at least accel (6B) + gyro (6B)

  auto stamp = imu_node_->get_clock()->now();

  // accel: int16 × 0.001 m/s²
  // gyro:  int16 × 0.001 rad/s
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp    = stamp;
  imu_msg.header.frame_id = "imu_link";

  imu_msg.linear_acceleration.x = get_le16(&p[0]) * 0.001;
  imu_msg.linear_acceleration.y = get_le16(&p[2]) * 0.001;
  imu_msg.linear_acceleration.z = get_le16(&p[4]) * 0.001;

  imu_msg.angular_velocity.x = get_le16(&p[6]) * 0.001;
  imu_msg.angular_velocity.y = get_le16(&p[8]) * 0.001;
  imu_msg.angular_velocity.z = get_le16(&p[10]) * 0.001;

  // Orientation not estimated — signal with -1 in first covariance element
  imu_msg.orientation_covariance[0] = -1.0;

  imu_pub_->publish(imu_msg);

  if (len < 18) return;  // need magn (6B)

  // magn: int16 × 0.1 µT → convert to T (1 µT = 1e-6 T)
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header.stamp    = stamp;
  mag_msg.header.frame_id = "imu_link";
  mag_msg.magnetic_field.x = get_le16(&p[12]) * 1e-7;
  mag_msg.magnetic_field.y = get_le16(&p[14]) * 1e-7;
  mag_msg.magnetic_field.z = get_le16(&p[16]) * 1e-7;

  mag_pub_->publish(mag_msg);
}

void Picar2Hardware::process_byte(uint8_t b)
{
  switch (decode_state_) {
    case DecodeState::START:
      if (b == PROTO_START) decode_state_ = DecodeState::TYPE;
      break;

    case DecodeState::TYPE:
      rx_type_      = b;
      decode_state_ = DecodeState::LEN;
      break;

    case DecodeState::LEN:
      if (b > PROTO_MAX_LEN) { decode_state_ = DecodeState::START; break; }
      rx_len_       = b;
      rx_pos_       = 0;
      decode_state_ = (b > 0) ? DecodeState::PAYLOAD : DecodeState::CRC;
      break;

    case DecodeState::PAYLOAD:
      rx_buf_[rx_pos_++] = b;
      if (rx_pos_ == rx_len_) decode_state_ = DecodeState::CRC;
      break;

    case DecodeState::CRC: {
      uint8_t crc_input[2 + PROTO_MAX_LEN];
      crc_input[0] = rx_type_;
      crc_input[1] = rx_len_;
      std::memcpy(&crc_input[2], rx_buf_, rx_len_);

      if (b == crc8(crc_input, 2 + rx_len_)) {
        if (rx_type_ == STREAM_JOINT && rx_len_ >= 10) {
          dispatch_joint_frame(rx_buf_, rx_len_);
        } else if (rx_type_ == STREAM_IMU && rx_len_ >= 12) {
          dispatch_imu_frame(rx_buf_, rx_len_);
        } else if (rx_type_ == MSG_TIMESYNC_RESP && rx_len_ >= 8) {
          dispatch_timesync_resp();
        }
      }
      decode_state_ = DecodeState::START;
      break;
    }
  }
}

// ── read — send timesync + request, wait for joint frame ─────────────────────
hardware_interface::return_type Picar2Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) return hardware_interface::return_type::OK;

  // Arm flag before sending request (timesync runs independently in timesync_loop)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    joint_ready_ = false;
  }

  // Send MSG_REQ for JOINT
  uint8_t req_payload[1] = {STREAM_JOINT};
  uint8_t req_frame[8];
  uart_write(req_frame, encode_frame(MSG_REQ, req_payload, 1, req_frame));

  // Wait for reader thread to deliver the JOINT frame
  std::unique_lock<std::mutex> lk(state_mutex_);
  bool got = joint_cv_.wait_for(lk, std::chrono::milliseconds(5),
                                 [this] { return joint_ready_; });
  if (!got) {
    lk.unlock();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "no JOINT response within 5ms");
    return hardware_interface::return_type::OK;
  }

  pos_back_left_  = stg_pos_left_;
  pos_back_right_ = stg_pos_right_;
  vel_back_left_  = stg_vel_left_;
  vel_back_right_ = stg_vel_right_;
  pos_steer_left_  = stg_steer_;
  pos_steer_right_ = stg_steer_;
  // Front wheels are passive — estimate rotation from rear average for visualization
  // Left wheel joint axis is mirrored in URDF, so negate its position
  double avg_rear = (pos_back_left_ + pos_back_right_) * 0.5;
  pos_front_left_wheel_  = -avg_rear;
  pos_front_right_wheel_ =  avg_rear;
  int64_t pi_us   = stg_pi_time_us_;
  int64_t t3      = stg_t3_us_;
  joint_ready_    = false;
  lk.unlock();

  if (pi_us != 0) {
    double lag_ms = (t3 - pi_us) * 1e-3;
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000,
      "sensor lag: %.2f ms", lag_ms);
  }

  return hardware_interface::return_type::OK;
}

// ── write — send CMD_VEL every cycle ─────────────────────────────────────────
hardware_interface::return_type Picar2Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (fd_ < 0) return hardware_interface::return_type::OK;

  auto to_dps = [](double rads) -> int16_t {
    double dps = rads * RAD_TO_DEG;
    return static_cast<int16_t>(
      std::clamp(dps, static_cast<double>(INT16_MIN), static_cast<double>(INT16_MAX)));
  };

  auto to_steer = [](double rad) -> int16_t {
    double tenths = std::round(rad * RAD_TO_DEG * 10.0);
    return static_cast<int16_t>(std::clamp(tenths, -900.0, 900.0));
  };

  double cmd_steer = (cmd_steer_left_ + cmd_steer_right_) * 0.5;

  uint8_t payload[6];
  put_le16(&payload[0], to_dps(cmd_vel_back_left_));
  put_le16(&payload[2], to_dps(cmd_vel_back_right_));
  put_le16(&payload[4], to_steer(cmd_steer));

  uint8_t frame[16];
  uart_write(frame, encode_frame(MSG_CMD_VEL, payload, sizeof(payload), frame));

  return hardware_interface::return_type::OK;
}

}  // namespace picar2_control

PLUGINLIB_EXPORT_CLASS(picar2_control::Picar2Hardware, hardware_interface::SystemInterface)
