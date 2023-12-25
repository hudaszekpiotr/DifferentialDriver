#ifndef MOTORS_DRIVER_CPP_HPP_
#define MOTORS_DRIVER_CPP_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/wheels_velocities.hpp" 

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

const std::chrono::milliseconds ODOMETRY_PERIOD{50};
const double TRACK_WIDTH = 0.122;

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode();
private:
    void new_odom_data_callback(const interfaces::msg::WheelsVelocities& msg);
    void publish_odom_callback();
    float x;
    float y;
    float x_velocity;
    float yaw_velocity;
    rclcpp::Time t_0;
    bool first_message;
    tf2::Quaternion orientation_quaternion;
    std::array<double, 36> pose_covariance;
    std::array<double, 36> twist_covariance;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<interfaces::msg::WheelsVelocities>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
};


#endif 
