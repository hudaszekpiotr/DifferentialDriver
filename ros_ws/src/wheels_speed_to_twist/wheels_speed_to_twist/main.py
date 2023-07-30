import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from interfaces.msg import WheelsVelocities

class WheelsSpeedToTwist(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.subscription = self.create_subscription(WheelsVelocities, 'actual_velocity', self.actual_velocity_callback, 10)
        self.subscription  # prevent unused variable warning
        self.actual_cmd_vel_publisher = self.create_publisher(Twist, 'actual_cmd_vel', 10)

    def actual_velocity_callback(self, msg):
        msg = Twist()
        actual_cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    wheels_speed_to_twist = WheelsSpeedToTwist()

    rclpy.spin(wheels_speed_to_twist)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wheels_speed_to_twist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
