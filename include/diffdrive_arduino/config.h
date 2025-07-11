#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 5;
  std::string device = "/dev/ttyACM0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 20;

  std::string imu_sensor_name = "imu_sensor";
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H