#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <map>
#include <thread>
#include <atomic>

// eCAL
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>

// Protobuf
#include "robot_state.pb.h"
#include "CompressedImage.pb.h"

using namespace enac;

enum class Source
{
    CAM,
    VIDEO,
    ECAL,
    GSTREAMER
};


class ArucoFinder
{
public:
    ArucoFinder(const std::string &name,
                const std::map<int, float> &arucos,
                bool display);

 
    double open_video(std::string src);
    void open_cam(std::string cam, int width = 0, int height = 0, double fps = 0, std::string fourcc  = "");
    void open_ecal(std::string topic);
    void open_gstreamer(std::string pipeline);

    void process(cv::Mat &frame);

    void readFrame(cv::Mat& frame);


private:
    std::string name;
    std::map<int, float> arucos;
    bool display;

    cv::VideoCapture cap;
    cv::Mat img;

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    cv::Ptr<cv::aruco::Dictionary> dictionary;

    std::unique_ptr<eCAL::protobuf::CPublisher<Arucos>> aruco_pub;
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::CompressedImage>> cam_pub;
    std::unique_ptr<eCAL::protobuf::CSubscriber<foxglove::CompressedImage>> sub;
};