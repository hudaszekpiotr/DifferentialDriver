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
        self.pi.set_glitch_filter(cliff_sensor_pin, 10000)
        self.sensor_publisher = self.create_publisher(Range, 'cliff_sensor', 10)
        self.pi.callback(cliff_sensor_pin, pigpio.EITHER_EDGE, self.cliff_sensor_pin_callback)
        msg = Range()
        msg.header.frame_id = "base_cliff_sensor"
        msg.radiation_type = 1
        msg.field_of_view = math.pi/6
        msg.min_range = 0.0
        msg.max_range = 0.016
        if self.pi.read(cliff_sensor_pin):
            msg.range = 0.001
        else:
            msg.range = 1.0
        self.msg = msg
        self.sensor_publisher.publish(self.msg)
        
    def cliff_sensor_pin_callback(self, gpio, level, tick):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        if level == 0:
            self.msg.range = 1.0
            self.sensor_publisher.publish(self.msg)
        elif level == 1:
            self.msg.range = 0.001
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
