#ifndef CLIFF_SENSOR_CPP_HPP_
#define CLIFF_SENSOR_CPP_HPP_

#include "interfaces/srv/label_face.hpp"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/face.hpp>
#include "std_msgs/msg/header.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

const std::string CSV_FILENAME = "data.csv";
std::string package_share_directory = ament_index_cpp::get_package_share_directory("startup");
std::string FACE_CASCADE_FILENAME = package_share_directory + "/config/haarcascade_frontalface_default.xml";

using namespace std::chrono_literals;


class FaceTrainer : public rclcpp::Node
{
public:
    FaceTrainer();
    void new_image_callback(const sensor_msgs::msg::Image& msg);
    void detect_faces(cv::Mat image);
    void pause_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void resume_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void label_callback(const std::shared_ptr<interfaces::srv::LabelFace::Request> request,
          std::shared_ptr<interfaces::srv::LabelFace::Response> response);
    void train_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void generate_csv_file();
private:
    bool pause = false;
    cv::Mat image;
    std::vector<cv::Rect> faces;
    cv::CascadeClassifier face_cascade;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_faces_publisher;
    std::string recognize_face(cv::Mat face_image);
};

#endif 
