import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from interfaces.msg import WheelsVelocities
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Quaternion
import math

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry')
        odometry_period = 0.05
        self.subscription = self.create_subscription(WheelsVelocities, 'actual_velocity', self.new_odom_data_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(odometry_period, self.publish_odom_callback)
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.x_velocity = 0.0
        self.yaw_velocity = 0.0
        self.orientation_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.t_0 = None
        self.track_width = 0.122
        # Row-major representation of the 6x6 covariance matrix
        # The orientation parameters use a fixed-axis representation.
        # In order, the parameters are:
        # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        self.pose_covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        self.twist_covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

    def new_odom_data_callback(self, msg):
        if self.t_0 is None:
            self.t_0 = self.get_clock().now()
            return 0
        self.x_velocity = (msg.right_wheel + msg.left_wheel) / 2
        self.yaw_velocity = (msg.right_wheel - msg.left_wheel) / self.track_width

        delta_time = self.get_clock().now() - self.t_0
        self.t_0 = self.get_clock().now()
        delta_time_sec = delta_time.nanoseconds / 10**9
        delta_yaw = self.yaw_velocity * delta_time_sec
        roll, pitch, yaw = euler_from_quaternion(self.orientation_quaternion)
        self.x = self.x + self.x_velocity * delta_time_sec * math.cos(yaw + delta_yaw / 2)
        self.y = self.y + self.x_velocity * delta_time_sec * math.sin(yaw + delta_yaw / 2)
        delta_quaternion = quaternion_from_euler(0.0, 0.0, -delta_yaw)
        self.orientation_quaternion = quaternion_multiply(delta_quaternion, self.orientation_quaternion)

    def publish_odom_callback(self):
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.child_frame_id = "base_link"

        pose_with_covariance = PoseWithCovariance()
        pose_with_covariance.pose.position.x = self.x
        pose_with_covariance.pose.position.y = self.y

        quaternion = Quaternion(x=self.orientation_quaternion[0], y=self.orientation_quaternion[1],
                              z=self.orientation_quaternion[2], w=self.orientation_quaternion[3])
        
        pose_with_covariance.pose.orientation = quaternion
        pose_with_covariance.covariance = self.pose_covariance
        odometry.pose = pose_with_covariance

        twist_with_covariance = TwistWithCovariance()
        twist_with_covariance.twist.linear.x = self.x_velocity
        twist_with_covariance.twist.angular.z = -self.yaw_velocity
        twist_with_covariance.covariance = self.twist_covariance
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
