#include "twist_to_wheels_speed_cpp/twist_to_wheels_speed_cpp.hpp"

using namespace std::chrono_literals;

TwistToWheelsSpeed::TwistToWheelsSpeed() : Node("twist_to_wheels_speed"){
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&TwistToWheelsSpeed::cmd_vel_callback, this, std::placeholders::_1));

    requested_velocity_publisher = this->create_publisher<interfaces::msg::WheelsVelocities>("requested_velocity", 10);
}

void TwistToWheelsSpeed::cmd_vel_callback(const geometry_msgs::msg::Twist& msg) const{
    auto wheels_velocities = interfaces::msg::WheelsVelocities();
    wheels_velocities.right_wheel = msg.linear.x - (msg.angular.z * TRACK_WIDTH) / 2;
    wheels_velocities.left_wheel = msg.linear.x + (msg.angular.z * TRACK_WIDTH) / 2;
    requested_velocity_publisher->publish(wheels_velocities);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToWheelsSpeed>());
    rclcpp::shutdown();
    return 0;
}
