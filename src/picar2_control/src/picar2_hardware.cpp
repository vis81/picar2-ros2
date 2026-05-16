#include "picar2_control/picar2_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace picar2_control
{

hardware_interface::CallbackReturn Picar2Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_ = info_.hardware_parameters.count("port")
    ? info_.hardware_parameters.at("port")
    : "/dev/ttyYahboom0";
  baud_ = info_.hardware_parameters.count("baud")
    ? std::stoi(info_.hardware_parameters.at("baud"))
    : 460800;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Picar2Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: open serial port, configure async read
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Picar2Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Picar2Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: send zero velocity, close serial port
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Picar2Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back("back_left_joint",        hardware_interface::HW_IF_POSITION, &pos_back_left_);
  interfaces.emplace_back("back_left_joint",        hardware_interface::HW_IF_VELOCITY, &vel_back_left_);
  interfaces.emplace_back("back_right_joint",       hardware_interface::HW_IF_POSITION, &pos_back_right_);
  interfaces.emplace_back("back_right_joint",       hardware_interface::HW_IF_VELOCITY, &vel_back_right_);
  interfaces.emplace_back("front_left_steer_joint", hardware_interface::HW_IF_POSITION, &pos_steer_left_);
  interfaces.emplace_back("front_right_steer_joint",hardware_interface::HW_IF_POSITION, &pos_steer_right_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Picar2Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back("back_left_joint",        hardware_interface::HW_IF_VELOCITY, &cmd_vel_back_left_);
  interfaces.emplace_back("back_right_joint",       hardware_interface::HW_IF_VELOCITY, &cmd_vel_back_right_);
  interfaces.emplace_back("front_left_steer_joint", hardware_interface::HW_IF_POSITION, &cmd_steer_);
  return interfaces;
}

hardware_interface::return_type Picar2Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: drain incoming frames from ring buffer, update pos_/vel_ from JOINT_STATE
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Picar2Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: encode and send CMD_VEL frame to STM32
  return hardware_interface::return_type::OK;
}

}  // namespace picar2_control

PLUGINLIB_EXPORT_CLASS(picar2_control::Picar2Hardware, hardware_interface::SystemInterface)
