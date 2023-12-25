#ifndef TWIST_TO_WHEELS_SPEED_CPP_HPP_
#define TWIST_TO_WHEELS_SPEED_CPP_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/wheels_velocities.hpp" 
#include "geometry_msgs/msg/twist.hpp"


const float TRACK_WIDTH = 0.122;

class TwistToWheelsSpeed : public rclcpp::Node
{
public:
    TwistToWheelsSpeed();
    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) const;
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::WheelsVelocities>::SharedPtr requested_velocity_publisher;
};


#endif 
