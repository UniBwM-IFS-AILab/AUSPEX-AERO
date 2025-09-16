#include "auspex_ocs/cam_publisher_mk_smart.hpp"

MkSmartCamPublisher::MkSmartCamPublisher(std::string platform_id, const int transmitHeight, const int transmitWidth, const float fps)
    : CamPublisherBase(platform_id, "mk_smart_cam_publisher", transmitHeight, transmitWidth, fps),
    stream_url_("rtsp://192.168.144.25:8554/video1"),
    capture_()
{
    std::string pipeline =
        "rtspsrc location=" + stream_url_ +
        " latency=200 ! rtph265depay ! h265parse ! avdec_h265 ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink sync=false drop=true max-buffers=1";

    capture_.open(pipeline, cv::CAP_GSTREAMER);
    if (!capture_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", stream_url_.c_str());
      return;
    }
}

MkSmartCamPublisher::~MkSmartCamPublisher()
{
    stopCapture();
    if (capture_.isOpened()) {
        capture_.release();
    }
}

void MkSmartCamPublisher::captureFrame()
{
    cv::Mat frame;
    bool isGrabbed = capture_.grab();
    if (!isGrabbed) {
        RCLCPP_ERROR(this->get_logger(), "cannot grab frame from RTSP stream.");
        return;
    }

    capture_.retrieve(frame);

    cv::resize(frame, frame, cv::Size(transmitWidth_, transmitHeight_), 0, 0, cv::INTER_LINEAR);

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);
    std::vector<uint8_t> jpg;
    if (!cv::imencode(".jpg", frame, jpg, compression_params)) {
        RCLCPP_ERROR(get_logger(), "JPG encoding failed");
        return;
    }

    auto image_msg = FrameData();
    auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    compressed_image_msg->header = std_msgs::msg::Header();
    compressed_image_msg->format = "jpeg";
    compressed_image_msg->data = jpg;
    image_msg.image_compressed = *compressed_image_msg;
    image_msg.platform_id = platform_id_;
    image_msg.team_id = "drone_team";
    image_msg.fps = fps_;
    image_msg.res_width = transmitWidth_;
    image_msg.res_height = transmitHeight_;

    if (gps_listener_) {
        auto g = gps_listener_->get_recent_gps_msg();
        image_msg.gps_position.latitude = g->latitude_deg;
        image_msg.gps_position.longitude = g->longitude_deg;
        image_msg.gps_position.altitude = g->absolute_altitude_m;
    }

    image_publisher_->publish(image_msg);

}