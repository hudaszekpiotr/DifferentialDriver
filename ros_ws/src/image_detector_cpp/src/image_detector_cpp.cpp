#include "image_detector_cpp/image_detector_cpp.hpp"

using namespace std::chrono_literals;


ImageDetector::ImageDetector() : Node("image_detector_node"), objPoints(4, 1, CV_32FC3){
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&ImageDetector::new_image_callback, this, std::placeholders::_1));

    detected_markers_publisher = this->create_publisher<visualization_msgs::msg::Marker>("detected_markers", 10);
    detected_markers_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("detected_markers_image", 10);

    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    if(!this->read_camera_parameters(CAMERA_PARAMS_FILENAME)){
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Error: camera params file not valid: " << CAMERA_PARAMS_FILENAME);
    }
    if(!face_cascade.load(FACE_CASCADE_FILENAME)){
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Error: face cascade file not valid: " << FACE_CASCADE_FILENAME);
    }
}

bool ImageDetector::read_camera_parameters(std::string filename){
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void ImageDetector::new_image_callback(const sensor_msgs::msg::Image& msg){
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, std::string());
    cv::Mat image = image_ptr->image;

    detect_aruco_markers(image);
    detect_faces(image);
    recognize_faces(image);
}

void ImageDetector::detect_aruco_markers(cv::Mat image){
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(image, aruco_dictionary, corners, ids, aruco_params);

    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
        int nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        // Calculate pose for each marker
        for (int i = 0; i < nMarkers; i++) {
            solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
        // Draw axis for each marker
        for (unsigned int i = 0; i < ids.size(); i++) {
            cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
    }

    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    detected_markers_image_publisher->publish(*image_msg.get());

    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_camera";
    marker.header.stamp = this->now();

    marker.ns = "aruco_markers";
    marker.id = 0;
    marker.type = SPHERE;
    marker.action = ADD;

    //marker.pose = ;
    geometry_msgs::msg::Vector3 vector;
    vector.x = 1;
    vector.y = 1;
    vector.z = 1;
    marker.scale = vector;

    std_msgs::msg::ColorRGBA color;
    color.r = 255;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker.color = color;

    rclcpp::Duration duration(1.0, 0.0);
    marker.lifetime = duration;
    marker.frame_locked = false;

    // marker.points = ;
    // marker.colors = ;

    // marker.text = "";
    // marker.mesh_resource = ;    // marker.mesh_use_embedded_materials = ;

    detected_markers_publisher->publish(marker);
}

void ImageDetector::detect_faces(cv::Mat image){
    cv::Mat image_gray;
    cv::cvtColor( image, image_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( image_gray, image_gray );

    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale( image_gray, faces );
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        cv::ellipse( image, center, cv::Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4 );
        //cv::Mat faceROI = frame_gray( faces[i] );
    }
}

void ImageDetector::recognize_faces(cv::Mat image){
    ;
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageDetector>());
    rclcpp::shutdown();
    return 0;
}
