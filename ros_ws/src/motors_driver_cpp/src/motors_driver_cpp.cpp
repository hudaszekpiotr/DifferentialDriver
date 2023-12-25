#include "motors_driver_cpp/motors_driver_cpp.hpp"

using namespace std::chrono_literals;


Motor::Motor(int pi, float pid_period, unsigned pwm_pin, unsigned direction_pin, unsigned encoder_A_pin, 
	unsigned encoder_B_pin, bool is_motor_reversed)
{
    this->pi = pi;
    this->pid_period = pid_period;
    this->pwm_pin = pwm_pin;
    this->direction_pin = direction_pin;
    this->is_motor_reversed = is_motor_reversed;
    this->encoder_A_pin = encoder_A_pin;
    this->encoder_B_pin = encoder_B_pin;
    requested_velocity = 0;
    actual_velocity = 0;
    current_tick = 0;
    last_tick = 0;
    error_integrated = 0;
    error_previous = 0;
    encoder_B_pin_state = 0;

    set_mode(pi, direction_pin, PI_OUTPUT);
    set_mode(pi, encoder_A_pin, PI_INPUT);
    set_glitch_filter(pi, encoder_A_pin, 100);
    set_mode(pi, encoder_B_pin, PI_INPUT);
    set_glitch_filter(pi, encoder_B_pin, 100);

    callback_ex(pi, encoder_A_pin, EITHER_EDGE, encoder_callback, this);
    callback_ex(pi, encoder_B_pin, EITHER_EDGE, encoder_callback, this);
}

void Motor::calculate_actual_velocity() {
    actual_velocity = ((double)(current_tick - last_tick) / NUMBER_OF_TICKS_PER_REVOLUTION) * WHEELS_DIAMETER * M_PI / pid_period;
    if (is_motor_reversed) {
        actual_velocity = -actual_velocity;
    }
    last_tick = current_tick;
}

float Motor::calculate_pid_effort() {
    float error = (requested_velocity - actual_velocity);

    error_integrated += error * pid_period;
    if (error_integrated > MAX_ERROR_INTEGRATED) {
        error_integrated = MAX_ERROR_INTEGRATED;
    }
    if (error_integrated < -MAX_ERROR_INTEGRATED) {
        error_integrated = -MAX_ERROR_INTEGRATED;
    }
    
    float error_derivative = error - error_previous;
    error_previous = error;

    float effort = P * error + I * error_integrated + D * error_derivative;
    if (requested_velocity == 0) {
        error_derivative = 0;
        effort = 0;
        error_integrated = 0;
        error_previous = 0;
    }
    

    if (effort > 1){
        effort = 1;
    }
    if (effort < -1) {
        effort = -1;
    }
            
    return effort;
}

void Motor::set_motor_effort(float effort) {
    unsigned dutycycle = 0;
    if(is_motor_reversed){
        effort = -effort;
    }
    if (effort <= 0) {
        gpio_write(pi, direction_pin, 0);
        dutycycle = int(std::abs(effort) * 1000000);
    } else {
        gpio_write(pi, direction_pin, 1);
        dutycycle = 1000000 - int(std::abs(effort) * 1000000);
    }
    hardware_PWM(pi, pwm_pin, PWM_FREQUENCY, dutycycle);
}

void Motor::stop() {
    hardware_PWM(pi, pwm_pin, PWM_FREQUENCY, 0);
}

void Motor::set_requested_velocity(float requested_velocity){
	this->requested_velocity = requested_velocity;
}

float Motor::get_actual_velocity(){
	return actual_velocity;
}



void encoder_callback([[maybe_unused]] int pi, unsigned gpio, unsigned level,[[maybe_unused]] unsigned tick, void * motor_void_ptr) {
    auto motor = static_cast<Motor*>(motor_void_ptr);
    if (gpio == motor->encoder_B_pin && level == 1) {
        motor->encoder_B_pin_state = 1;
    }
    if (gpio == motor->encoder_B_pin && level == 0) {
        motor->encoder_B_pin_state = 0;
    }

    if (gpio == motor->encoder_A_pin && level == 1) {
        if (motor->encoder_B_pin_state) {
            motor->current_tick -= 1;
        }
        else {
            motor->current_tick += 1;
        }
    }

    if (gpio == motor->encoder_A_pin && level == 0) {
        if (motor->encoder_B_pin_state) {
            motor->current_tick += 1;
        }
        else {
            motor->current_tick -= 1;
        }
    }
}


MotorsDriver::MotorsDriver() : Node("motors_driver"){
    pi = pigpio_start(NULL, NULL);
	float pid_period_seconds = std::chrono::duration_cast<std::chrono::duration<float>>(PID_PERIOD).count();

    right_motor = std::make_unique<Motor>(pi, pid_period_seconds, RIGHT_PWM_PIN, RIGHT_DIRECTION_PIN, RIGHT_ENCODER_FORWARD_PIN, RIGHT_ENCODER_BACK_PIN, RIGHT_MOTOR_REVERSED);
    left_motor = std::make_unique<Motor>(pi, pid_period_seconds, LEFT_PWM_PIN, LEFT_DIRECTION_PIN, LEFT_ENCODER_FORWARD_PIN, LEFT_ENCODER_BACK_PIN, LEFT_MOTOR_REVERSED);


    subscription_ = this->create_subscription<interfaces::msg::WheelsVelocities>(
            "requested_velocity", 10, std::bind(&MotorsDriver::set_requested_speed_callback, this, std::placeholders::_1));
    actual_velocity_publisher = this->create_publisher<interfaces::msg::WheelsVelocities>("actual_velocity", 10);
    timer_ = this->create_wall_timer(
        PID_PERIOD, std::bind(&MotorsDriver::calculate_pid_callback, this));
}


void MotorsDriver::set_requested_speed_callback(const interfaces::msg::WheelsVelocities& msg) {
    right_motor->set_requested_velocity(msg.right_wheel);
    left_motor->set_requested_velocity(msg.left_wheel);
}

void MotorsDriver::calculate_pid_callback() {
    float effort;
    right_motor->calculate_actual_velocity();
    effort = right_motor->calculate_pid_effort();
    right_motor->set_motor_effort(effort);

    left_motor->calculate_actual_velocity();
    effort = left_motor->calculate_pid_effort();
    left_motor->set_motor_effort(effort);

    auto msg = interfaces::msg::WheelsVelocities();
    msg.right_wheel = right_motor->get_actual_velocity();
    msg.left_wheel = left_motor->get_actual_velocity();
    actual_velocity_publisher->publish(msg);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorsDriver>());
    rclcpp::shutdown();
    return 0;
}
