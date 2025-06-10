#include "auspex_ocs/cam_publisher_sim.hpp"

SimCamPublisher::SimCamPublisher(const std::string& platform_id, const float fps)
    : CamPublisherBase(platform_id, "sim_cam_publisher", fps)
{
    vhcl_id_ = std::stoi(platform_id.substr(platform_id.rfind('_')+1));
    sim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>();
}

SimCamPublisher::~SimCamPublisher() {
    stopCapture();
}

void SimCamPublisher::captureFrame() {
    // build request
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> reqs = {
        {"front_30", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false}
    };
    auto responses = sim_client_->simGetImages(reqs, "Drone" + std::to_string(vhcl_id_+1));
    auto& raw = responses[0].image_data_uint8;
    if (raw.empty()) {
        RCLCPP_ERROR(get_logger(), "AirSim returned an empty image");
        return;
    }

    int w = responses[0].width;
    int h = responses[0].height;
    cv::Mat img(h, w, CV_8UC3, raw.data());
    cv::Mat rgba;
    cv::cvtColor(img, rgba, cv::COLOR_RGB2RGBA);

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);
    std::vector<uint8_t> jpg;
    if (!cv::imencode(".jpg", rgba, jpg, compression_params)) {
        RCLCPP_ERROR(get_logger(), "JPG encoding failed");
        return;
    }

    FrameData msg;
    msg.platform_id = platform_id_;
    msg.team_id = "drone_team";
    msg.image_compressed.data = std::move(jpg);
    msg.image_compressed.header.stamp = get_clock()->now();
    msg.image_compressed.format = "jpeg";
    msg.fps = static_cast<int>(fps_);
    msg.res_width = w;
    msg.res_height = h;

    if (gps_listener_) {
        auto g = gps_listener_->get_recent_gps_msg();
        msg.gps_position.latitude = g->lat;
        msg.gps_position.longitude = g->lon;
        msg.gps_position.altitude = g->alt;
    }

    image_publisher_->publish(msg);
}