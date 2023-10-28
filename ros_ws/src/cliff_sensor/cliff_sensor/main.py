import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math

cliff_sensor_pin = 17
period = 0.05

class CliffSensorNode(Node):

    def __init__(self):
        super().__init__('motors_driver')
        self.pi = pigpio.pi()
        self.pi.set_mode(cliff_sensor_pin, pigpio.INPUT)
        
        self.timer = self.create_timer(period, self.read_pin_callback)
        self.sensor_publisher = self.create_publisher(Range, 'cliff_sensor', 10)
        
        msg = Range()
        msg.header.frame_id = "base_cliff_sensor"
        msg.radiation_type = 1
        msg.field_of_view = math.pi/6
        msg.min_range = 0.0
        msg.max_range = 0.02
        self.msg = msg
        
    def read_pin_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        level = self.pi.read(cliff_sensor_pin)
        if level:
            self.msg.range = 0.0
        else:
            self.msg.range = 1.0

        self.sensor_publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    cliff_sensor_node = CliffSensorNode()

    rclpy.spin(cliff_sensor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cliff_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
