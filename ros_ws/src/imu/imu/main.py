import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html

import rclpy
from rclpy.node import Node
from interfaces.msg import WheelsVelocities
from sensor_msgs.msg import Imu
import math
import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html
import sys


accelerometer_control_register = 0x10 #hex

# in Hz
accelerometer_frequency = {
    0:      0b0000,
    1.6:    0b1011,
    12.5:   0b0001,
    26:     0b0010,
    52:     0b0011,
    104:    0b0100,
    208:    0b0101,
    416:    0b0110,
    833:    0b0111,
    1660:   0b1000,
    3330:   0b1001,
    6660:   0b1010
}

# in g
accelerometer_full_scale = {
    2:      0b00,
    16:     0b01,
    4:      0b10,
    8:      0b11
}

accelerometer_filtering_type = {
    "first_stage":  0b00,
    "second_stage": 0b10
}

gyroscope_control_register = 0x11 #hex

# in Hz
gyroscope_frequency = {
    0:      0b0000,
    12.5:   0b0001,
    26:     0b0010,
    52:     0b0011,
    104:    0b0100,
    208:    0b0101,
    416:    0b0110,
    833:    0b0111,
    1660:   0b1000,
    3330:   0b1001,
    6660:   0b1010
}

# in degres per second(dps)
gyroscope_full_scale = {
    125:    0b0010,
    250:    0b0000,
    500:    0b0100,
    1000:   0b1000,
    2000:   0b1100
}

temp_out_register = 0x20
angular_x_register = 0x22
angular_y_register = 0x24
angular_z_register = 0x26
linear_x_register = 0x28
linear_y_register = 0x2A
linear_z_register = 0x2C

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return -val

class ImuDriver():
    def __init__(self):
        self.pi = pigpio.pi()
        self.handle = self.pi.i2c_open(1, 0x6B)
        self.accelerometer_control_register_data = 0b00000000
        self.gyroscope_control_register_data =     0b00000000
        self.accelerometer_full_scale = 0
        self.gyroscope_full_scale = 0

    def set_accelerometer_frequency(self, frequency):
        register = self.accelerometer_control_register_data
        register = register & 0b1111
        register = register | accelerometer_frequency[frequency]<<4
        self.accelerometer_control_register_data = register

    def set_accelerometer_full_scale(self, scale):
        register = self.accelerometer_control_register_data
        register = register & 0b11110011
        register = register | accelerometer_full_scale[scale]<<2
        self.accelerometer_control_register_data = register
        self.accelerometer_full_scale = scale

    def set_accelerometer_filtering_type(self, type):
        register = self.accelerometer_control_register_data
        register = register & 0b11111100
        register = register | accelerometer_filtering_type[type]
        self.accelerometer_control_register_data = register

    def set_gyroscope_frequency(self, frequency):
        register = self.gyroscope_control_register_data
        register = register & 0b1111
        register = register | gyroscope_frequency[frequency]<<4
        self.gyroscope_control_register_data = register

    def set_gyroscope_full_scale(self, scale):
        register = self.gyroscope_control_register_data
        register = register & 0b11110000
        register = register | gyroscope_full_scale[scale]
        self.gyroscope_control_register_data = register
        self.gyroscope_full_scale = scale

    def configure(self):
        self.pi.i2c_write_byte_data(self.handle, accelerometer_control_register, self.accelerometer_control_register_data)
        self.pi.i2c_write_byte_data(self.handle, gyroscope_control_register, self.gyroscope_control_register_data)

    def read_angular_velocity_x(self):
        value = self.pi.i2c_read_word_data(self.handle, angular_x_register)
        value = twos_comp(value, 16) / 32767 * self.gyroscope_full_scale * math.pi / 180
        return value

    def read_angular_velocity_y(self):
        value = self.pi.i2c_read_word_data(self.handle, angular_y_register)
        value = twos_comp(value, 16) / 32767 * self.gyroscope_full_scale * math.pi / 180
        return value

    def read_angular_velocity_z(self):
        value = self.pi.i2c_read_word_data(self.handle, angular_z_register)
        value = twos_comp(value, 16) / 32767 * self.gyroscope_full_scale * math.pi / 180
        return value

    def read_linear_acceleration_x(self):
        value = self.pi.i2c_read_word_data(self.handle, linear_x_register)
        value = twos_comp(value, 16) / 32767 * self.accelerometer_full_scale * 9.807
        return value

    def read_linear_acceleration_y(self):
        value = self.pi.i2c_read_word_data(self.handle, linear_y_register)
        value = twos_comp(value, 16) / 32767 * self.accelerometer_full_scale * 9.807
        return value

    def read_linear_acceleration_z(self):
        value = self.pi.i2c_read_word_data(self.handle, linear_z_register)
        value = twos_comp(value, 16) / 32767 * self.accelerometer_full_scale * 9.807
        return value

    def read_temp(self):
        pass

class ImuNode(Node):
    def __init__(self):
        super().__init__('motors_driver')
        imu_period = 0.05  # seconds
        self.timer = self.create_timer(imu_period, self.publish_imu_callback)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        imu_driver = ImuDriver()
        imu_driver.set_accelerometer_frequency(104)
        imu_driver.set_accelerometer_full_scale(2)
        imu_driver.set_accelerometer_filtering_type("first_stage")

        imu_driver.set_gyroscope_frequency(104)
        imu_driver.set_gyroscope_full_scale(125)
        imu_driver.configure()
        self.imu_driver = imu_driver


    def publish_imu_callback(self):
        imu = Imu()
        imu.header.frame_id = "imu_link"
        imu.header.stamp = self.get_clock().now().to_msg()

        imu.angular_velocity.x = self.imu_driver.read_angular_velocity_x()
        imu.angular_velocity.y = self.imu_driver.read_angular_velocity_y()
        imu.angular_velocity.z = self.imu_driver.read_angular_velocity_z()
        imu.angular_velocity_covariance

        imu.linear_acceleration.x = self.imu_driver.read_linear_acceleration_x()
        imu.linear_acceleration.y = self.imu_driver.read_linear_acceleration_y()
        imu.linear_acceleration.z = self.imu_driver.read_linear_acceleration_z()
        imu.linear_acceleration_covariance
        
        self.imu_publisher.publish(imu)


def main(args=None):
    rclpy.init(args=args)

    imu_node = ImuNode()

    rclpy.spin(imu_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
