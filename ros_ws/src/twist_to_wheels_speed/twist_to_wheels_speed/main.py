import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from interfaces.msg import WheelsVelocities

class TwistToWheelsSpeed(Node):

    def __init__(self):
        super().__init__('twist_to_wheels_speed')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.subscription  # prevent unused variable warning
        self.requested_velocity_publisher = self.create_publisher(WheelsVelocities, 'requested_velocity', 10)

    def cmd_vel_callback(self, msg):
        msg = WheelsVelocities()
        #msg.right_wheel =
        #msg.left_wheel =
        self.requested_velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    twist_to_wheels_speed = TwistToWheelsSpeed()

    rclpy.spin(twist_to_wheels_speed)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_to_wheels_speed.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
