import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math

trigger_pin = 16
echo_pin = 19
period = 0.1

class UltrasonicSensorNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_sensor')
        self.pi = pigpio.pi()
        self.pi.set_mode(echo_pin, pigpio.INPUT)
        self.pi.set_mode(trigger_pin, pigpio.OUTPUT)
        
        self.timer = self.create_timer(period, self.trigger_pin_callback)
        self.pi.callback(echo_pin, pigpio.EITHER_EDGE, self.echo_pin_callback)

        self.sensor_publisher = self.create_publisher(Range, 'ultrasonic_sensor', 10)
        
        msg = Range()
        msg.header.frame_id = "base_ultrasonic_sensor"
        msg.radiation_type = 0
        msg.field_of_view = math.pi/6
        msg.min_range = 0.02
        msg.max_range = 3.0
        self.msg = msg
        self.tick_start = None
        
    def trigger_pin_callback(self):
        self.pi.gpio_trigger(trigger_pin, 10, 1)

    def echo_pin_callback(self, gpio, level, tick):
        if level == 1:
            self.tick_start = tick
        elif level == 0 and self.tick_start is not None:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            tick_diff = tick - self.tick_start
            if tick_diff < 0:
                tick_diff += (1 << 32)
            self.msg.range = (tick_diff / 1000000 * 343) / 2
            self.sensor_publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    ultrasonic_sensor_node = UltrasonicSensorNode()

    rclpy.spin(ultrasonic_sensor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ultrasonic_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
