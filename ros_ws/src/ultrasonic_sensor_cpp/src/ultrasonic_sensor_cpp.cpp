#include "ultrasonic_sensor_cpp/ultrasonic_sensor_cpp.hpp"

using namespace std::chrono_literals;

UltrasonicSensorNode::UltrasonicSensorNode(): Node("ultrasonic_sensor"){
    sensor_publisher = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic_sensor", 10);
    timer_ = this->create_wall_timer(
        NODE_PERIOD, std::bind(&UltrasonicSensorNode::trigger_pin_callback, this));

    pi = pigpio_start(NULL, NULL);
    set_mode(pi, ECHO_PIN, PI_INPUT);
    set_glitch_filter(pi, ECHO_PIN, 100);
    set_mode(pi, TRIGGER_PIN, PI_OUTPUT);
    callback_ex(pi, ECHO_PIN, EITHER_EDGE, echo_pin_callback, this);

    msg.header.frame_id = "base_ultrasonic_sensor";
    msg.radiation_type = 0;
    msg.field_of_view = FIELD_OF_VIEW;
    msg.min_range = MIN_RANGE;
    msg.max_range = MAX_RANGE;
    start = true;
    tick_start = 0;
}

void UltrasonicSensorNode::trigger_pin_callback() {
    gpio_trigger(pi, TRIGGER_PIN, 10, 1);
}

void echo_pin_callback([[maybe_unused]] int pi, [[maybe_unused]] unsigned gpio, unsigned level, unsigned tick, void* node_void_ptr) {
    auto node = static_cast<UltrasonicSensorNode*>(node_void_ptr);
    float tick_diff = 0;
    if (level == 1) {
        node->tick_start = tick;
        node->start = false;
    }
    else if (level == 0 && !node->start) {
        node->msg.header.stamp = node->now();
            tick_diff = tick - node->tick_start;

            if (tick_diff < 0) {
                tick_diff = tick + std::numeric_limits<uint32_t>::max() - node->tick_start;
            }
        node->msg.range = (tick_diff / 1000000 * SPEED_OF_SOUND) / 2;
            node->sensor_publisher->publish(node->msg);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicSensorNode>());
    rclcpp::shutdown();
    return 0;
}
