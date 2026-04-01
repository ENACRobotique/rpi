#include "arucoFinder.h"
#include "string.h"

ArucoFinder::ArucoFinder(const std::string &name,
            const std::map<int, float> &arucos,
            bool display)
    : name(name), arucos(arucos), display(display)
{
    eCAL::Initialize("arucoFinder");

    aruco_pub = std::make_unique<eCAL::protobuf::CPublisher<Arucos>>("Arucos");

    if (display) {
        cam_pub = std::make_unique<eCAL::protobuf::CPublisher<foxglove::CompressedImage>>("images_" + name);
    }

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

}

double ArucoFinder::open_video(std::string src){
    cap.open(src);
    
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open source\n";
        exit(1);
    }
    auto fps = cap.get(cv::CAP_PROP_FPS);
    auto w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    auto h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "opening video " << src << " at " << w << "x" << h << ", " << fps << " fps" << std::endl;
    return fps;
}

void ArucoFinder::open_cam(std::string cam, int width, int height, double fps,  std::string fourcc){
    cap.open(cam, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open source\n";
        exit(1);
    }
    if (!fourcc.empty()){
        if (fourcc.length() == 4){
            
            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
        } else {
            printf("Tu as fait de la merde, il faut 4 lettres dans FOURcc !\n");
        }
    }
    if (width != 0){
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    }
    if (height != 0){
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }
    if (fps != 0){
        cap.set(cv::CAP_PROP_FPS, fps);
    }
    
    fps = cap.get(cv::CAP_PROP_FPS);
    width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    int read_fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));

    char fourcc_str[] = {
        (char)(read_fourcc & 0xFF),
        (char)((read_fourcc >> 8) & 0xFF),
        (char)((read_fourcc >> 16) & 0xFF),
        (char)((read_fourcc >> 24) & 0xFF),
        '\0'
    };

    std::cout << "opening cam " << cam << " at " << width << "x" << height << ", " << fps << " fps, fourcc : "  << fourcc_str << std::endl;
}

void ArucoFinder::open_ecal(std::string topic)
{
    sub = std::make_unique<eCAL::protobuf::CSubscriber<foxglove::CompressedImage>>(topic);
    sub->SetReceiveCallback([this](const eCAL::STopicId &publisher_id_, const foxglove::CompressedImage &msg, long long time_, long long clock_)
        {
            std::vector<uint8_t> data(msg.data().begin(), msg.data().end());
            img = cv::imdecode(data, cv::IMREAD_COLOR);
            process(img);
        }
    );
}

void ArucoFinder::open_gstreamer(std::string pipeline){
    std::cout << "opening gstreamer "<< std::endl;
    cap.open(pipeline, cv::CAP_GSTREAMER);
}

void ArucoFinder::process(cv::Mat& frame)
{
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    cv::aruco::detectMarkers(frame, dictionary, corners, ids);

    if (display)
    {
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf);

        foxglove::CompressedImage msg;
        msg.set_data(buf.data(), buf.size());
        msg.set_format("jpeg");

        cam_pub->Send(msg);
    }

    if (!ids.empty())
    {
        Arucos msg;

        for (size_t i = 0; i < ids.size(); ++i)
        {
            int id = ids[i];

            if (arucos.find(id) == arucos.end())
                continue;

            float size = arucos.at(id);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                std::vector<std::vector<cv::Point2f>>{corners[i]},
                //corners[i],
                size,
                camera_matrix,
                dist_coeffs,
                rvecs,
                tvecs);

            if (!tvecs.empty())
            {
                auto *ar = msg.add_arucos();

                ar->set_x(tvecs[0][0]);
                ar->set_y(tvecs[0][1]);
                ar->set_z(tvecs[0][2]);
                ar->set_arucoid(id);

                // rotation → quaternion
                cv::Mat R;
                cv::Rodrigues(rvecs[0], R);

                // conversion simple (approx)
                double qw = std::sqrt(1 + R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2)) / 2;
                double qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) / (4 * qw);
                double qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) / (4 * qw);
                double qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) / (4 * qw);

                ar->set_qx(qx);
                ar->set_qy(qy);
                ar->set_qz(qz);
                ar->set_qw(qw);
            }
        }

        msg.set_cameraname(name);
        aruco_pub->Send(msg);
    }
}

void ArucoFinder::readFrame(cv::Mat& frame){
    cap >> frame;
}