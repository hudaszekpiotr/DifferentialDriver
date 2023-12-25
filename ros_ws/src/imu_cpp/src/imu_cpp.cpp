#include "imu_cpp/imu_cpp.hpp"

using namespace std::chrono_literals;


ImuDriver::ImuDriver(){
	pi = pigpio_start(NULL, NULL);
	handle = i2c_open(pi, 1, 0x6B, 0);
	accelerometer_control_register_data = 0b00000000;
	gyroscope_control_register_data = 0b00000000;
	accelerometer_full_scale_selected = 0;
	gyroscope_full_scale_selected = 0;
}

void ImuDriver::set_accelerometer_frequency(int frequency) {
    accelerometer_control_register_data &= 0b1111;
    accelerometer_control_register_data |= accelerometer_frequency[frequency] << 4;
}

void ImuDriver::set_accelerometer_full_scale(int scale) {
    accelerometer_control_register_data &= 0b11110011;
    accelerometer_control_register_data |= accelerometer_full_scale[scale] << 2;
    accelerometer_full_scale_selected = scale;
}

void ImuDriver::set_accelerometer_filtering_type(std::string type) {
    accelerometer_control_register_data &= 0b11111100;
    accelerometer_control_register_data |= accelerometer_filtering_type[type];
}

void ImuDriver::set_gyroscope_frequency(int frequency) {
    gyroscope_control_register_data &= 0b1111;
    gyroscope_control_register_data |= gyroscope_frequency[frequency] << 4;
}

void ImuDriver::set_gyroscope_full_scale(int scale) {
    gyroscope_control_register_data &= 0b11110000;
    gyroscope_control_register_data |= gyroscope_full_scale[scale];
    gyroscope_full_scale_selected = scale;
}

void ImuDriver::configure() {
    i2c_write_byte_data(pi, handle, ACCELEROMETER_CONTROL_REGISTER, accelerometer_control_register_data);
    i2c_write_byte_data(pi, handle, GYROSCOPE_CONTROL_REGISTER, gyroscope_control_register_data);
}

float ImuDriver::read_angular_velocity_x() {
    int16_t value = i2c_read_word_data(pi, handle, ANGULAR_X_REGISTER);
    float value_conv = (float)value / 32767 * gyroscope_full_scale_selected * M_PI / 180;
    return -value_conv;
}

float ImuDriver::read_angular_velocity_y() {
    int16_t value = i2c_read_word_data(pi, handle, ANGULAR_Y_REGISTER);
    float value_conv = (float)value / 32767 * gyroscope_full_scale_selected * M_PI / 180;
    return -value_conv;
}

float ImuDriver::read_angular_velocity_z() {
    int16_t value = i2c_read_word_data(pi, handle, ANGULAR_Z_REGISTER);
    float value_conv = (float)value / 32767 * gyroscope_full_scale_selected * M_PI / 180;
    return -value_conv;
}

float ImuDriver::read_linear_acceleration_x() {
    int16_t value = i2c_read_word_data(pi, handle, LINEAR_X_REGISTER);
    float value_conv = (float)value / 32767 * accelerometer_full_scale_selected * 9.807;
    return -value_conv;
}

float ImuDriver::read_linear_acceleration_y() {
    int16_t value = i2c_read_word_data(pi, handle, LINEAR_Y_REGISTER);
    float value_conv = (float)value / 32767 * accelerometer_full_scale_selected * 9.807;
    return -value_conv;
}

float ImuDriver::read_linear_acceleration_z() {
    int16_t value = i2c_read_word_data(pi, handle, LINEAR_Z_REGISTER);
    float value_conv = (float)value / 32767 * accelerometer_full_scale_selected * 9.807;
    return -value_conv;
}


ImuNode::ImuNode(): Node("imu_node"){
    imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    timer_ = this->create_wall_timer(
        IMU_PERIOD, std::bind(&ImuNode::publish_imu_callback, this));

    imu_driver.set_accelerometer_frequency(SELECTED_ACCEL_FREQUENCY);
    imu_driver.set_accelerometer_full_scale(SELECTED_ACCEL_FULL_SCALE);
    imu_driver.set_accelerometer_filtering_type(SELECTED_ACCEL_FILTERING_TYPE);

    imu_driver.set_gyroscope_frequency(SELECTED_GYROSCOPE_FREQUENCY);
    imu_driver.set_gyroscope_full_scale(SELECTED_GYROSCOPE_FULL_SCALE);
    imu_driver.configure();

    msg.header.frame_id = "imu_link";
    msg.header.stamp = this->now();

    msg.orientation_covariance = { -1.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0 };
    msg.angular_velocity_covariance = { 0.1, 0.0, 0.0,
                                        0.0, 0.1, 0.0,
                                        0.0, 0.0, 0.1 };
    msg.linear_acceleration_covariance = { 0.1, 0.0, 0.0,
                                           0.0, 0.1, 0.0,
                                           0.0, 0.0, 0.1 };
}

void ImuNode::publish_imu_callback()
{
    msg.header.stamp = this->now();

    msg.angular_velocity.x = imu_driver.read_angular_velocity_x();
    msg.angular_velocity.y = imu_driver.read_angular_velocity_y();
    msg.angular_velocity.z = imu_driver.read_angular_velocity_z();
    
    msg.linear_acceleration.x = imu_driver.read_linear_acceleration_x();
    msg.linear_acceleration.y = imu_driver.read_linear_acceleration_y();
    msg.linear_acceleration.z = imu_driver.read_linear_acceleration_z();
    
    imu_publisher->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
