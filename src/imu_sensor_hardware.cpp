#include "diffdrive_arduino/imu_sensor.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diffdrive_arduino
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

ImuSensorHardware::ImuSensorHardware()
    : logger_(rclcpp::get_logger("ImuSensorHardware"))
{}

CallbackReturn ImuSensorHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.imu_sensor_name = info_.hardware_parameters["imu_sensor"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ImuSensorHardware::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Orientation (Quaternion - typically handled separately)
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "orientation.x", &imu_sensor_.quat[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "orientation.y", &imu_sensor_.quat[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "orientation.z", &imu_sensor_.quat[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "orientation.w", &imu_sensor_.quat[3]));

  // Angular Velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "angular_velocity.x", &imu_sensor_.gyro[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "angular_velocity.y", &imu_sensor_.gyro[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "angular_velocity.z", &imu_sensor_.gyro[2]));

  // Linear Acceleration
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "linear_acceleration.x", &imu_sensor_.accel[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "linear_acceleration.y", &imu_sensor_.accel[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_sensor_.name, "linear_acceleration.z", &imu_sensor_.accel[2]));

  return state_interfaces;
}

// std::vector<hardware_interface::CommandInterface> ImuSensorHardware::export_command_interfaces()
// {
//     return {};  // IMU usually has no commands
// }

CallbackReturn ImuSensorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting Sensor...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuSensorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping Sensor...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ImuSensorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &)
{
  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.readIMUValues(imu_sensor_.accel, imu_sensor_.gyro, imu_sensor_.quat);

  return return_type::OK;

  
}

// hardware_interface::return_type ImuSensorHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
// {
//     return hardware_interface::return_type::OK;  // IMU is read-only
// }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::ImuSensorHardware,
  hardware_interface::SensorInterface
)

