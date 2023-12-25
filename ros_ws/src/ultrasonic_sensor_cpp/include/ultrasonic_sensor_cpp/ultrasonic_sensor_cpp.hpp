#ifndef ULTRASONIC_SENSOR_CPP_HPP_
#define ULTRASONIC_SENSOR_CPP_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <limits>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

const std::chrono::milliseconds NODE_PERIOD{ 100 };
const unsigned ECHO_PIN = 19;
const unsigned TRIGGER_PIN = 16;
const float FIELD_OF_VIEW = M_PI / 6;
const float MIN_RANGE = 0.02;
const float MAX_RANGE = 3.0;
const float SPEED_OF_SOUND = 343;

using namespace std::chrono_literals;


class UltrasonicSensorNode : public rclcpp::Node
{
public:
    UltrasonicSensorNode();
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sensor_publisher;
    sensor_msgs::msg::Range msg;
    unsigned tick_start;
    bool start;
private:
    int pi;
    void trigger_pin_callback();
    rclcpp::TimerBase::SharedPtr timer_;
};

void echo_pin_callback([[maybe_unused]] int pi, unsigned gpio, unsigned level, unsigned tick, void* node_void_ptr);

#endif 
