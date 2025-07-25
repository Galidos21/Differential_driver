#ifndef DIFFDRIVE_IMU_REAL_ROBOT_H
#define DIFFDRIVE_IMU_REAL_ROBOT_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "config.h"
#include "imu.h"
#include "arduino_comms.h"

using hardware_interface::return_type;

namespace diffdrive_arduino
{

class ImuSensorHardware : public hardware_interface::SensorInterface
{

public:
  ImuSensorHardware();
  
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  //std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  //return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;



private:

  Config cfg_;

  ArduinoComms arduino_;
  
  ImuSensor imu_sensor_;

  rclcpp::Logger logger_;

  std::chrono::time_point<std::chrono::system_clock> time_;
  
};

}

#endif // DIFFDRIVE_IMU_REAL_ROBOT_H