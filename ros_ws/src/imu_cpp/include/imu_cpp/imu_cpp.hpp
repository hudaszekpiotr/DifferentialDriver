#ifndef IMU_CPP_HPP_
#define IMU_CPP_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include "sensor_msgs/msg/imu.hpp"

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const std::chrono::milliseconds IMU_PERIOD{ 50 };
unsigned SELECTED_ACCEL_FREQUENCY = 104;
unsigned SELECTED_ACCEL_FULL_SCALE = 2;
std::string SELECTED_ACCEL_FILTERING_TYPE = "first_stage";
unsigned SELECTED_GYROSCOPE_FREQUENCY = 104;
unsigned SELECTED_GYROSCOPE_FULL_SCALE = 125;

std::unordered_map<unsigned, uint8_t> accelerometer_frequency = {
    {0,  0b0000},
    {2, 0b1011},
    {13, 0b0001},
    {26, 0b0010},
    {52, 0b0011},
    {104, 0b0100},
    {208, 0b0101},
    {416, 0b0110},
    {833, 0b0111},
    {1660, 0b1000},
    {3330, 0b1001},
    {6660, 0b1010}
};

// in g
std::unordered_map<unsigned, uint8_t> accelerometer_full_scale = {
    {2, 0b00},
    {16, 0b01},
    {4, 0b10},
    {8, 0b11}
};

std::unordered_map<std::string, uint8_t> accelerometer_filtering_type = {
    {"first_stage", 0b00},
    {"second_stage", 0b10}
};



// in Hz
std::unordered_map<int, uint8_t> gyroscope_frequency = {
    {0,  0b0000},
    {13, 0b0001},
    {26, 0b0010},
    {52, 0b0011},
    {104, 0b0100},
    {208, 0b0101},
    {416, 0b0110},
    {833, 0b0111},
    {1660, 0b1000},
    {3330, 0b1001},
    {6660, 0b1010}
};

// in degres per second(dps)
std::unordered_map<int, uint8_t> gyroscope_full_scale = {
    {125, 0b0010},
    {250, 0b0000},
    {500, 0b0100},
    {1000, 0b1000},
    {2000, 0b1100}
};

unsigned ACCELEROMETER_CONTROL_REGISTER = 0x10;
unsigned GYROSCOPE_CONTROL_REGISTER = 0x11;
unsigned TEMP_OUT_REGISTER = 0x20;
unsigned ANGULAR_X_REGISTER = 0x22;
unsigned ANGULAR_Y_REGISTER = 0x24;
unsigned ANGULAR_Z_REGISTER = 0x26;
unsigned LINEAR_X_REGISTER = 0x28;
unsigned LINEAR_Y_REGISTER = 0x2A;
unsigned LINEAR_Z_REGISTER = 0x2C;


class ImuDriver {
public:
    ImuDriver();
    void set_accelerometer_frequency(int frequency);
    void set_accelerometer_full_scale(int scale);
    void set_accelerometer_filtering_type(std::string type);
    void set_gyroscope_frequency(int frequency);
    void set_gyroscope_full_scale(int scale);
    void configure();
    float read_angular_velocity_x();
    float read_angular_velocity_y();
    float read_angular_velocity_z();
    float read_linear_acceleration_x();
    float read_linear_acceleration_y();
    float read_linear_acceleration_z();
private:
    int pi;
    unsigned handle;
    unsigned accelerometer_control_register_data;
    unsigned gyroscope_control_register_data;
    unsigned accelerometer_full_scale_selected;
    unsigned gyroscope_full_scale_selected;
};



class ImuNode : public rclcpp::Node
{
public:
    ImuNode();
private:
    void publish_imu_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    sensor_msgs::msg::Imu msg;
    ImuDriver imu_driver;
};

#endif 
