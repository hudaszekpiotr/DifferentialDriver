#include<iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/aruco.hpp>

cv::Ptr<cv::aruco::Dictionary> aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

void generate_marker() {
    int markerId;
    std::cout<<"Input marker ID: ";
    std::cin>>markerId;
    int markerSize;
    std::cout<<"Input marker size in px, default is 200: ";
    std::cin>>markerSize;

    cv::Mat markerImg;
    cv::aruco::drawMarker(aruco_dictionary, markerId, markerSize, markerImg);

    std::stringstream filename;
    filename<<"Marker_Id"<<markerId<<"_Size"<<markerSize<<".png";
    cv::imwrite(filename.str(), markerImg);
    std::cout<<"File created: "<<filename.str();
}

int main(){
    generate_marker();
}
