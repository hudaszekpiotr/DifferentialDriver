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


const float WHEELS_DIAMETER = 0.079; //in meters
const int NUMBER_OF_TICKS_PER_REVOLUTION = 960 * 2; //we are counting both edges
const std::chrono::milliseconds PID_PERIOD{50};

const unsigned PWM_FREQUENCY = 1000;
const float MAX_ERROR_INTEGRATED = 0.1;
const float P = 2.0;
const float I = 10.0;
const float D = 0.0;

const unsigned RIGHT_PWM_PIN = 13;
const unsigned RIGHT_DIRECTION_PIN = 25;
const unsigned RIGHT_ENCODER_FORWARD_PIN = 18;
const unsigned RIGHT_ENCODER_BACK_PIN = 27;
const bool RIGHT_MOTOR_REVERSED = false;

const unsigned LEFT_PWM_PIN = 12;
const unsigned LEFT_DIRECTION_PIN = 24;
const unsigned LEFT_ENCODER_FORWARD_PIN = 22;
const unsigned LEFT_ENCODER_BACK_PIN = 23;
const bool LEFT_MOTOR_REVERSED = true;


class Motor {
public:
    Motor(int pi, float pid_period, unsigned pwm_pin, unsigned direction_pin, unsigned encoder_A_pin, 
    	unsigned encoder_B_pin, bool is_motor_reversed);
    void calculate_actual_velocity();
    float calculate_pid_effort();
    void set_motor_effort(float effort);
    void stop();
    void set_requested_velocity(float requested_velocity);
    float get_actual_velocity();
    unsigned encoder_A_pin;
    unsigned encoder_B_pin;
    long current_tick;
    bool encoder_B_pin_state;
private:
    int pi;
    float pid_period;
    unsigned pwm_pin;
    unsigned direction_pin;
    bool is_motor_reversed;
    float requested_velocity;
    float actual_velocity;
    long last_tick;
    float error_integrated;
    float error_previous;
};


void encoder_callback([[maybe_unused]] int pi, unsigned gpio, unsigned level,[[maybe_unused]] unsigned tick, void * motor);

class MotorsDriver : public rclcpp::Node
{
public:
    MotorsDriver();
private:
    int pi;
    std::unique_ptr<Motor> right_motor;
    std::unique_ptr<Motor> left_motor;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::WheelsVelocities>::SharedPtr actual_velocity_publisher;
    rclcpp::Subscription<interfaces::msg::WheelsVelocities>::SharedPtr subscription_;
    void set_requested_speed_callback(const interfaces::msg::WheelsVelocities& msg);
    void calculate_pid_callback();
};



#endif 

