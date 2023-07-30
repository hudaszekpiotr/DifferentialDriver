import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html

import rclpy
from rclpy.node import Node
from interfaces.msg import WheelsVelocities
from sensor_msgs.msg import Imu
import math


class ImuNode(Node):
    def __init__(self):
        super().__init__('motors_driver')
        imu_period = 0.05  # seconds
        self.timer = self.create_timer(imu_period, self.publish_imu_callback)
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)

    def publish_imu_callback(self):
        imu = Imu()
        imu.header.frame_id = "imu"
        imu.header.stamp = self.get_clock().now().to_msg()

        imu.orientation
        imu.orientation_covariance

        imu.angular_velocity
        imu.angular_velocity_covariance

        imu.linear_acceleration
        imu.linear_acceleration_covariance
        
        self.imu_publisher.publish(msg)


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
