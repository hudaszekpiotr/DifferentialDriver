import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from interfaces.msg import WheelsVelocities
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Quaternion
import math

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry')
        odometry_period = 0.01
        self.subscription = self.create_subscription(WheelsVelocities, 'actual_velocity', self.new_odom_data_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(odometry_period, self.publish_odom_callback)
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.x_velocity = 0.0
        self.yaw_velocity = 0.0
        self.yaw_angle = 0.0
        self.t_0 = None
        self.track_width = 0.122
        # Row-major representation of the 6x6 covariance matrix
        # The orientation parameters use a fixed-axis representation.
        # In order, the parameters are:
        # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        self.pose_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def new_odom_data_callback(self, msg):
        if self.t_0 is None:
            self.t_0 = self.get_clock().now()
            return 0
        self.x_velocity = (msg.right_wheel + msg.left_wheel) / 2
        self.yaw_velocity = (msg.right_wheel - msg.left_wheel) / self.track_width

        delta_time = self.get_clock().now() - self.t_0
        delta_time_sec = delta_time.nanoseconds / 10**9
        delta_yaw = self.yaw_velocity * delta_time_sec
        self.x = self.x + self.x_velocity * delta_time_sec * math.cos(self.yaw_angle + delta_yaw / 2)
        self.y = self.y + self.x_velocity * delta_time_sec * math.sin(self.yaw_angle + delta_yaw / 2)
        self.yaw_angle = self.yaw_angle + delta_yaw

    def publish_odom_callback(self):
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.child_frame_id = "base_link"

        pose_with_covariance = PoseWithCovariance()
        pose_with_covariance.pose.position.x = self.x
        pose_with_covariance.pose.position.y = self.y

        quaternion = Quaternion()
        (x, y, z, w) = quaternion_from_euler(0.0, 0.0, self.yaw_angle)
        quaternion.x = x
        quaternion.y = y
        quaternion.z = z
        quaternion.w = w
        
        pose_with_covariance.pose.orientation = Quaternion()
        pose_with_covariance.covariance = self.pose_covariance
        odometry.pose = pose_with_covariance

        twist_with_covariance = TwistWithCovariance()
        twist_with_covariance.twist.linear.x = self.x_velocity
        twist_with_covariance.twist.angular.z = self.yaw_velocity
        #twist_with_covariance.covariance =
        odometry.twist = twist_with_covariance
        self.odometry_publisher.publish(odometry)


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
