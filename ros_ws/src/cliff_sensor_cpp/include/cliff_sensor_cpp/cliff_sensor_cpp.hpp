#ifndef CLIFF_SENSOR_CPP_HPP_
#define CLIFF_SENSOR_CPP_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

unsigned CLIFF_SENSOR_PIN = 17;
float FIELD_OF_VIEW = M_PI / 6;
float MIN_RANGE = 0.0;
float MAX_RANGE = 0.016;

using namespace std::chrono_literals;


class CliffSensorNode : public rclcpp::Node
{
public:
    CliffSensorNode();
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sensor_publisher;
    sensor_msgs::msg::Range msg;
private:
    int pi;
    rclcpp::TimerBase::SharedPtr timer_;
};

void cliff_sensor_pin_callback([[maybe_unused]] int pi, unsigned gpio, unsigned level, unsigned tick, void* node_void_ptr);

#endif 
