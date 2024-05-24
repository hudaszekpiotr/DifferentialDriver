#ifndef IMAGE_DETECTOR_CPP_HPP_
#define IMAGE_DETECTOR_CPP_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include "std_msgs/msg/header.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/aruco.hpp>

float markerLength = 0.05;
std::string package_share_directory = ament_index_cpp::get_package_share_directory("startup");
std::string CAMERA_PARAMS_FILENAME = package_share_directory + "/config/image_detector_aruco.yaml";
cv::Ptr<cv::aruco::Dictionary> aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Ptr<cv::aruco::DetectorParameters> aruco_params = cv::aruco::DetectorParameters::create();

cv::String FACE_CASCADE_FILENAME = "";

enum MarkerType {
    ARROW,
    CUBE,
    SPHERE,
    CYLINDER,
    LINE_STRIP,
    LINE_LIST,
    CUBE_LIST,
    SPHERE_LIST,
    POINTS,
    TEXT_VIEW_FACING,
    MESH_RESOURCE,
    TRIANGLE_LIST,
};

enum MarkerAction {
    ADD,
    MODIFY,
    DELETE,
    DELETEALL,
};

class ImageDetector : public rclcpp::Node
{
public:
    ImageDetector();
    void new_image_callback(const sensor_msgs::msg::Image& msg);
private:
    bool read_camera_parameters(std::string filename);
    void detect_aruco_markers(cv::Mat image);
    void detect_faces(cv::Mat image);
    void recognize_faces(cv::Mat image);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detected_markers_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_markers_image_publisher;
    cv::Mat objPoints;
    cv::Mat cameraMatrix, distCoeffs;
    cv::CascadeClassifier face_cascade;
};

cv::Mat cameraMatrix, distCoeffs;

#endif
