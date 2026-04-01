#include <CLI/CLI.hpp>
#include <iostream>
#include <map>
#include <thread>
#include <atomic>
#include "arucoFinder.h"

const std::string GST_PIPELINE =
    "udpsrc port=5000 caps=\"application/x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000\" ! "
    "rtph265depay ! avdec_h265 ! videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=1 sync=false max-buffers=1";


int main(int argc, char** argv) {
    CLI::App app{"Aruco Finder"};

    std::string name;
    std::string cam;
    std::string video;
    bool gstreamer = false;
    std::string topic;
    bool display = false;
    double fps = 0;
    int width = 0;
    int height = 0;
    std::string fourcc;

    app.add_option("name", name)->required();
    app.add_option("-c,--cam", cam);
    app.add_option("-v,--video", video);
    app.add_flag("-g,--gstreamer", gstreamer);
    app.add_option("-t,--topic", topic);
    app.add_flag("-d,--display", display);
    app.add_option("-W,--width", width);
    app.add_option("-H,--height", height);
    app.add_option("-f,--fps", fps);
    app.add_option("--fourcc", fourcc);

    CLI11_PARSE(app, argc, argv);


    std::map<int, float> arucos = {
        {47,30}, {36,30}, {20,100}, {21,100},
        {22,100}, {23,100}, {13,30}
    };

    ArucoFinder af(name, arucos, display);


    if (!cam.empty()) {
        af.open_cam(cam, width, height, fps, fourcc);
    } else if (!video.empty()) {
        fps = af.open_video(video);
    } else if (!topic.empty()) {
        af.open_ecal(topic);
    } else if (gstreamer) {
        af.open_gstreamer(GST_PIPELINE);
    } else {
        std::cerr << "No source specified\n";
        return 1;
    }
    


    auto last_time = std::chrono::steady_clock::now();

    while (eCAL::Ok())
    {
        if (topic.empty())
        {
            cv::Mat frame;
            af.readFrame(frame);
            if (frame.empty()) {
                exit(0);
            }
            af.process(frame);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if(fps != 0){
            int ms = 1000.0/fps;
            auto target_time = last_time + std::chrono::milliseconds(ms);
            std::this_thread::sleep_until(target_time);
            last_time = target_time;
        }
    }
}