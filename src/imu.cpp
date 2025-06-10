#include "diffdrive_arduino/imu.h"

ImuSensor::ImuSensor(const std::string &sensor_name)
{
  setup(sensor_name);
}


void ImuSensor::setup(const std::string &sensor_name)
{
  name = sensor_name;
}