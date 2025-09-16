#include "auspex_ocs/cam_publisher_base.hpp"

CamPublisherBase::CamPublisherBase(const std::string& platform_id, const std::string& node_name, const int transmitHeight, const int transmitWidth, float fps)
    : Node(platform_id + "_" + node_name),
    fps_(fps),
    platform_id_(platform_id),
    transmitWidth_(transmitWidth),
    transmitHeight_(transmitHeight)
{
    rmw_qos_profile_t sensor_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(
        rclcpp::QoSInitialization(sensor_profile.history, sensor_profile.depth),
        sensor_profile
    );
    image_publisher_ = this->create_publisher<FrameData>(platform_id + "/raw_camera_stream", sensor_qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / fps)),
        std::bind(&CamPublisherBase::onTimer, this)
    );
    timer_->cancel();
}

void CamPublisherBase::startCapture(std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener) {
    if (!isCapturing()) {
        gps_listener_ = gps_listener;
        timer_->reset();
        RCLCPP_INFO(this->get_logger(), "Camera started.");
    }
}

void CamPublisherBase::stopCapture() {
    if (isCapturing()) {
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Camera stopped.");

        FrameData stop_msg;
        stop_msg.image_compressed.format = "empty";
        image_publisher_->publish(stop_msg);
    }
}

bool CamPublisherBase::isCapturing() const {
    return !timer_->is_canceled();
}

void CamPublisherBase::onTimer() {
    if (capture_mutex_.try_lock()) {
        captureFrame();
        capture_mutex_.unlock();
    }
    // If mutex is locked, return immediately without blocking
}
