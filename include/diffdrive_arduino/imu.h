#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <string>

class ImuSensor {
public:
    std::string name = "";  // IMU sensor name
    double accel[3] = {0, 0, 0};   // Accelerometer (X, Y, Z)
    double gyro[3] = {0, 0, 0};    // Gyroscope (X, Y, Z)
    double quat[4] = {0, 0, 0, 0}; // Quaternion (w, x, y, z)

    ImuSensor() = default;  // Default constructor

    ImuSensor(const std::string &sensor_name);  // Constructor with sensor name

    void setup(const std::string &sensor_name);  // Setup method to initialize the sensor

    //void updateData(double ax, double ay, double az, double gx, double gy, double gz, double qw, double qx, double qy, double qz);

    //void printData();  // Method to print IMU data (for debugging)
};

#endif // IMU_SENSOR_H