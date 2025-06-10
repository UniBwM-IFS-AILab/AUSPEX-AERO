#include "auspex_ocs/cam_publisher_mk_smart.hpp"

MkSmartCamPublisher::MkSmartCamPublisher(std::string platform_id, const float fps)
    : CamPublisherBase(platform_id, "mk_smart_cam_publisher", fps),
    stream_url_("rtsp://192.168.144.25:8554/video1"),
    capture_()
{
}

MkSmartCamPublisher::~MkSmartCamPublisher()
{
    stopCapture();
    if (capture_.isOpened()) {
        capture_.release();
    }
}

void MkSmartCamPublisher::startCapture(std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener)
{
    capture_.open(stream_url_, cv::CAP_FFMPEG);
    if (!capture_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", stream_url_.c_str());
      return;
    }
    CamPublisherBase::startCapture(gps_listener);
}

void MkSmartCamPublisher::stopCapture()
{
    CamPublisherBase::stopCapture();
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


    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);
    std::vector<uint8_t> jpg;
    if (!cv::imencode(".jpg", frame, jpg, compression_params)) {
        RCLCPP_ERROR(get_logger(), "JPG encoding failed");
        return;
    }

    auto image_msg = FrameData();
    auto image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", jpg).toCompressedImageMsg();
    image_msg.platform_id = platform_id_;
    image_msg.team_id = "drone_team";
    image_msg.image_compressed = *image;
    image_msg.image_compressed.format = "jpeg";
    image_msg.fps = fps_;
    image_msg.res_width = 1280;
    image_msg.res_height = 720;

    if (gps_listener_) {
        auto g = gps_listener_->get_recent_gps_msg();
        msg.gps_position.latitude = g->lat;
        msg.gps_position.longitude = g->lon;
        msg.gps_position.altitude = g->alt;
    }

    image_publisher_->publish(image_msg);

}