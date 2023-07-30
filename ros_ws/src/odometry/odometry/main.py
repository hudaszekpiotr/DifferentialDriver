import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from interfaces.msg import WheelsVelocities

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry')
        odometry_period = 0.01
        self.subscription = self.create_subscription(WheelsVelocities, 'actual_velocity', self.new_odom_data_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(odometry_period, self.publish_odom_callback)
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)

    def new_odom_data_callback(self, msg):
    	pass
        #msg.right_wheel
        #msg.left_wheel

    def publish_odom_callback(self):
    	pass
        #msg = Odometry()
        #self.odometry_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    odometry = OdometryNode()

    rclpy.spin(odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
