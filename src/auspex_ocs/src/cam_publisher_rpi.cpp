#include "auspex_ocs/cam_publisher_rpi.hpp"

RPICamPublisher::RPICamPublisher(const std::string& platform_id, const float fps)
    : CamPublisherBase(platform_id, "rpi_cam_publisher", fps)
{
    initCamera();
}

RPICamPublisher::~RPICamPublisher() {
    // ensure timer stopped and release camera
    stopCapture();
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void RPICamPublisher::initCamera() {
    cap_.open(0);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera");
        rclcpp::shutdown();
    }
}

void RPICamPublisher::captureFrame() {
    cv::Mat frame;
    if (!cap_.read(frame)) {
        RCLCPP_WARN(get_logger(), "Failed to read frame");
        return;
    }

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);
    std::vector<uint8_t> jpg;
    if (!cv::imencode(".jpg", frame, jpg, compression_params)) {
        RCLCPP_ERROR(get_logger(), "JPG encoding failed");
        return;
    }

    // Populate FrameData
    FrameData image_msg;
    image_msg.platform_id = platform_id_;
    image_msg.team_id     = "drone_team";
    image_msg.image_compressed.data   = std::move(jpg);
    image_msg.image_compressed.header.stamp = get_clock()->now();
    image_msg.image_compressed.format = "jpeg";
    image_msg.fps        = static_cast<int>(fps_);
    image_msg.res_width  = frame.cols;
    image_msg.res_height = frame.rows;

    if (gps_listener_) {
        auto g = gps_listener_->get_recent_gps_msg();
        image_msg.gps_position.latitude  = g->latitude_deg;
        image_msg.gps_position.longitude = g->longitude_deg;
        image_msg.gps_position.altitude  = g->absolute_altitude_m;
    }

    image_publisher_->publish(image_msg);
}
