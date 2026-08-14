#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
  enc_ticks_per_rev_ = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);

  std::string left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  std::string right_wheel_name = info_.hardware_parameters["right_wheel_name"];

  l_wheel_.setup(left_wheel_name, enc_ticks_per_rev_);
  r_wheel_.setup(right_wheel_name, enc_ticks_per_rev_);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating serial link to ESP32...");
  try
  {
    comms_.connect(device_, baud_rate_, timeout_ms_);
  }
  catch (std::exception & e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Failed to connect to serial port '%s': %s", device_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully connected to ESP32.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating serial link...");
  comms_.disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(l_wheel_.enc, r_wheel_.enc);

  double prev_pos_l = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calc_enc_angle();
  l_wheel_.vel = (l_wheel_.pos - prev_pos_l) / period.seconds();

  double prev_pos_r = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calc_enc_angle();
  r_wheel_.vel = (r_wheel_.pos - prev_pos_r) / period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Convert rad/s to encoder ticks/sec or raw command
  int motor_l_val = static_cast<int>(l_wheel_.cmd / l_wheel_.rads_per_tick);
  int motor_r_val = static_cast<int>(r_wheel_.cmd / r_wheel_.rads_per_tick);

  comms_.set_motor_values(motor_l_val, motor_r_val);
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffBotSystemHardware, hardware_interface::SystemInterface)
