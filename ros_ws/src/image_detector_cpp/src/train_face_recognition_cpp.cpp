#include "image_detector_cpp/train_face_recognition_cpp.hpp"

using namespace std::chrono_literals;

FaceTrainer::FaceTrainer() : Node("face_trainer_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&ImageDetector::new_image_callback, this, std::placeholders::_1));

    detected_faces_publisher = this->create_publisher<sensor_msgs::msg::Image>("detected_faces_image", 10);

    if(!face_cascade.load(FACE_CASCADE_FILENAME)){
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Error: face cascade file not valid: " << FACE_CASCADE_FILENA>    }

    rclcpp::Service<interfaces::srv::LabelFace>::SharedPtr label_service =
    	this->create_service<interfaces::srv::LabelFace>("label_face", this.label_callback);
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_service =
        this->create_service<std_srvs::srv::Empty>("pause_face_detection", this.pause_callback);
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resume_service =
        this->create_service<std_srvs::srv::Empty>("resume_face_detection", this.resume_callback);
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

    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale( image_gray, faces );

    for(auto face: faces){
        cv::Point center( face.x + face.width/2, face.y + face.height/2 );
        cv::ellipse( image, center, cv::Size( face.width/2, face.height/2 ), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4 );
        extracted_face = frame_gray( face );

        std::string label = recognize_face(extracted_face);
        //draw label or unrecognized1
    }
    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    detected_faces_publisher->publish(*image_msg.get());
}

void FaceTrainer::pause_callback(){
    pause = true;
}

void FaceTrainer::resume_callback(){
    pause = false;
}

void FaceTrainer::label_callback(const std::shared_ptr<interfaces::srv::LabelFace::Request> request,
          std::shared_ptr<interfaces::srv::LabelFace::Response> response)
{
    cv::Mat image_gray;
    cv::cvtColor( image, image_gray, cv::COLOR_BGR2GRAY );
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale( image_gray, faces );

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream filename;
    filename << "data/" << request->label << "/" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".jpg";
    imwrite(filename, faces[request->id]);

    response->status = "ok";
}

void FaceTrainer::train(){
    generate_csv_file();

    try {
        read_csv(CSV_FILENAME, images, labels);
    } catch (const cv::Exception& e) {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        return;
    }
    // Quit if there are not enough images for this demo.
    if(images.size() <= 1) {
        string error_message = "This demo needs at least 2 images to work. Please add more images >        CV_Error(Error::StsError, error_message);
    }
    Ptr<LBPHFaceRecognizer> model = LBPHFaceRecognizer::create();
    model->train(images, labels);
}

void FaceTrainer::generate_csv_file(){
    std::filesystem::remove(CSV_FILENAME);
    std::ofstream csv_file(CSV_FILENAME);
    unsigned number = 0;

    for(auto& directory : std::filesystem::recursive_directory_iterator("data")){
        if (directory.is_directory()){
            for(auto& file : std::filesystem::recursive_directory_iterator(directory)){
                if(file.is_file()){
                    csv_file << file.path().string() << ";" << number << "\n";
                }
            }
        }
    }
    csv_file.close();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageDetector>());
    rclcpp::shutdown();
    return 0;
}
