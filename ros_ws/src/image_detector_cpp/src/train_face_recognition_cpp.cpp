#include "image_detector_cpp/train_face_recognition_cpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

bool read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, char separator = ';') {
    std::ifstream file(filename.c_str(), std::ifstream::in);
    if (!file) {
        return false;
    }
    std::string line, path, classlabel;
    while (getline(file, line)) {
        std::stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(cv::imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
    return true;
}


FaceTrainer::FaceTrainer() : Node("face_trainer_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&FaceTrainer::new_image_callback, this, _1));

    detected_faces_publisher = this->create_publisher<sensor_msgs::msg::Image>("detected_faces_image", 10);

    if(!face_cascade.load(FACE_CASCADE_FILENAME)){
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Error: face cascade file not valid: " << FACE_CASCADE_FILENAME);
    }

    rclcpp::Service<interfaces::srv::LabelFace>::SharedPtr label_service =
        this->create_service<interfaces::srv::LabelFace>("label_face", std::bind(&FaceTrainer::label_callback, this, _1, _2));
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_service =
        this->create_service<std_srvs::srv::Empty>("pause_face_detection", std::bind(&FaceTrainer::pause_callback, this, _1, _2));
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resume_service =
        this->create_service<std_srvs::srv::Empty>("resume_face_detection", std::bind(&FaceTrainer::resume_callback, this, _1, _2));
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr train_service =
        this->create_service<std_srvs::srv::Empty>("train_face_detection", std::bind(&FaceTrainer::train_callback, this, _1, _2));
}


void FaceTrainer::new_image_callback(const sensor_msgs::msg::Image& msg){
    if(pause){
        return;
    }

    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, std::string());
    image = image_ptr->image;
    detect_faces(image);
}

void FaceTrainer::detect_faces(cv::Mat image){
    cv::Mat image_gray;
    cv::cvtColor( image, image_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( image_gray, image_gray );

    face_cascade.detectMultiScale( image_gray, faces );

    for(auto face: faces){
        cv::Point center( face.x + face.width/2, face.y + face.height/2 );
        cv::ellipse( image, center, cv::Size( face.width/2, face.height/2 ), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4 );
        cv::Mat extracted_face = image_gray(face);

        std::string label = this->recognize_face(extracted_face);
        //draw label or unrecognized1
    }
    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    detected_faces_publisher->publish(*image_msg.get());
}

void FaceTrainer::pause_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response){
    pause = true;
}

void FaceTrainer::resume_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response){
    pause = false;
}

void FaceTrainer::label_callback(const std::shared_ptr<interfaces::srv::LabelFace::Request> request,
          std::shared_ptr<interfaces::srv::LabelFace::Response> response)
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream filename;
    filename << "data/" << request->label << "/" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".jpg";
    cv::imwrite(filename.str(), image(faces[request->id]));

    response->success = true;
}

void FaceTrainer::train_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response){
    std::vector<cv::Mat> images;
    std::vector<int> labels;
    generate_csv_file();

    auto success = read_csv(CSV_FILENAME, images, labels);
    if(!success){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error opening file " << CSV_FILENAME);
        return;
    }

    if(images.size() <= 1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "To train face recognition you need at least 2 images");
        return;
    }
    cv::Ptr<cv::face::LBPHFaceRecognizer> model = cv::face::LBPHFaceRecognizer::create();
    model->train(images, labels);
}

void FaceTrainer::generate_csv_file(){
    std::filesystem::remove(CSV_FILENAME);
    std::ofstream csv_file(CSV_FILENAME);
    unsigned number = 0;

    for(auto& directory : std::filesystem::recursive_directory_iterator("data")){
        if (directory.is_directory()){
            for(auto& file : std::filesystem::recursive_directory_iterator(directory)){
                if(file.is_regular_file()){
                    csv_file << file.path().string() << ";" << number << "\n";
                }
            }
        }
    }
    csv_file.close();
}

std::string FaceTrainer::recognize_face(cv::Mat face_image){
   return "test";
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceTrainer>());
    rclcpp::shutdown();
    return 0;
}
