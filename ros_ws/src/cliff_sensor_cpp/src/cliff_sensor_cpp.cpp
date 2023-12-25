#include "cliff_sensor_cpp/cliff_sensor_cpp.hpp"

using namespace std::chrono_literals;


CliffSensorNode::CliffSensorNode(): Node("cliff_sensor_node"){
    sensor_publisher = this->create_publisher<sensor_msgs::msg::Range>("cliff_sensor", 10);

    pi = pigpio_start(NULL, NULL);
    set_mode(pi, CLIFF_SENSOR_PIN, PI_INPUT);
    set_glitch_filter(pi, CLIFF_SENSOR_PIN, 10000);
    callback_ex(pi, CLIFF_SENSOR_PIN, EITHER_EDGE, cliff_sensor_pin_callback, this);

    msg.header.frame_id = "base_cliff_sensor";
    msg.radiation_type = 1;
    msg.field_of_view = FIELD_OF_VIEW;
    msg.min_range = MIN_RANGE;
    msg.max_range = MAX_RANGE;
    if (gpio_read(pi, CLIFF_SENSOR_PIN)) {
        msg.range = 0.001;
    }
    else {
        msg.range = 1.0;
    }
    sensor_publisher->publish(msg);
}

void cliff_sensor_pin_callback([[maybe_unused]] int pi, [[maybe_unused]] unsigned gpio, unsigned level, [[maybe_unused]] unsigned tick, void* node_void_ptr) {
    auto node = static_cast<CliffSensorNode*>(node_void_ptr);
    node->msg.header.stamp = node->now();
    if (level == 0) {
        node->msg.range = 1.0;
        node->sensor_publisher->publish(node -> msg);
    }
    else if (level == 1) {
        node->msg.range = 0.001;
        node->sensor_publisher->publish(node ->msg);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CliffSensorNode>());
    rclcpp::shutdown();
    return 0;
}
