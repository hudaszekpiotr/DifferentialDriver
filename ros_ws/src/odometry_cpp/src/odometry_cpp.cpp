#include "odometry_cpp/odometry_cpp.hpp"

using namespace std::chrono_literals;


OdometryNode::OdometryNode() : Node("odometry"){
    subscription_ = this->create_subscription<interfaces::msg::WheelsVelocities>(
        "actual_velocity", 10, std::bind(&OdometryNode::new_odom_data_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        ODOMETRY_PERIOD, std::bind(&OdometryNode::publish_odom_callback, this));
    odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    x = 0.0;
    y = 0.0;
    x_velocity = 0.0;
    yaw_velocity = 0.0;
    orientation_quaternion.setRPY(0, 0, 0);
    first_message = true;

    // Row - major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed - axis representation.
    // In order, the parameters are :
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    pose_covariance = { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1 };

    twist_covariance = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.01 };
}

void OdometryNode::new_odom_data_callback(const interfaces::msg::WheelsVelocities& msg){
    if (first_message) {
        t_0 = this->now();
        first_message = false;
    }else{
    	x_velocity = (msg.right_wheel + msg.left_wheel) / 2;
	    yaw_velocity = (msg.right_wheel - msg.left_wheel) / TRACK_WIDTH;

	    double delta_time = (this->now() - t_0).seconds();
	    t_0 = this->now();
	    double delta_yaw = yaw_velocity * delta_time;

	    tf2::Matrix3x3 matrix(orientation_quaternion);
	    double roll, pitch, yaw;
	    matrix.getRPY(roll, pitch, yaw);

	    x = x + x_velocity * delta_time * cos(yaw + delta_yaw / 2);
	    y = y + x_velocity * delta_time * sin(yaw + delta_yaw / 2);
	    tf2::Quaternion delta_quaternion;
	    delta_quaternion.setRPY(0.0, 0.0, -delta_yaw);
	    orientation_quaternion = delta_quaternion * orientation_quaternion;
	    orientation_quaternion.normalize();
    }
}
    
void OdometryNode::publish_odom_callback(){
    auto odometry = nav_msgs::msg::Odometry();
    odometry.header.frame_id = "odom";
    odometry.header.stamp = this->now();
    odometry.child_frame_id = "base_link";

    auto pose_with_covariance = geometry_msgs::msg::PoseWithCovariance();
    pose_with_covariance.pose.position.x = x;
    pose_with_covariance.pose.position.y = y;

    pose_with_covariance.pose.orientation = tf2::toMsg(orientation_quaternion);
    pose_with_covariance.covariance = pose_covariance;
    odometry.pose = pose_with_covariance;

    auto twist_with_covariance = geometry_msgs::msg::TwistWithCovariance();
    twist_with_covariance.twist.linear.x = x_velocity;
    twist_with_covariance.twist.angular.z = -yaw_velocity;
    twist_with_covariance.covariance = twist_covariance;
    odometry.twist = twist_with_covariance;

    odometry_publisher->publish(odometry);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
