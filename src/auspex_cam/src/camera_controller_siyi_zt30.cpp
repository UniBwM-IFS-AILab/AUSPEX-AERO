#include "auspex_cam/camera_controller_siyi_zt30.hpp"

CameraControllerSiyiZT30::CameraControllerSiyiZT30(const std::string& platform_id,
                                                   const std::string& ip_addr, int port,
                                                   int stream_width_color, int stream_height_color, int stream_bitrate_color,
                                                   int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                                                   const float fps)
    : CameraControllerBase(platform_id, "camera_controller_siyi_zt30", ip_addr, port, stream_width_color, stream_height_color, stream_bitrate_color, stream_width_ir, stream_height_ir, stream_bitrate_ir, fps),
    stream_url_color_("rtsp://192.168.144.25:8554/video1"),
    capture_color_(),
    stream_url_ir_("rtsp://192.168.144.25:8554/video2"),
    capture_ir_(),
    camera_()
{
    std::string pipeline =
        "rtspsrc location=" + stream_url_color_ +
        " latency=200 ! rtph265depay ! h265parse ! avdec_h265 ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink sync=false drop=true max-buffers=1";
    capture_color_.open(pipeline, cv::CAP_GSTREAMER);
    if (!capture_color_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open %s", stream_url_color_.c_str());
        return;
    }
    pipeline =
        "rtspsrc location=" + stream_url_ir_ +
        " latency=200 ! rtph265depay ! h265parse ! avdec_h265 ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink sync=false drop=true max-buffers=1";
    capture_ir_.open(pipeline, cv::CAP_GSTREAMER);
    if (!capture_ir_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open %s", stream_url_ir_.c_str());
        return;
    }
    
    camera_.set_gimbal_mode_lock();
    camera_.enable_lrf();
    camera_.start_lrf_dist_caching();
    camera_.start_gimbal_attitude_caching(5);
}

CameraControllerSiyiZT30::~CameraControllerSiyiZT30()
{
    if (capture_color_.isOpened()) {
        capture_color_.release();
    }
    if (capture_ir_.isOpened()) {
        capture_ir_.release();
    }
    camera_.disable_lrf();
    camera_.stop_lrf_dist_caching();
    camera_.stop_gimbal_attitude_caching();
}

CameraControllerBase::Capabilities CameraControllerSiyiZT30::available_capabilities() {
    return Capabilities{ true, true, true, true };
}

bool CameraControllerSiyiZT30::check_if_connected()
{
    return static_cast<bool>(camera_.get_hardware_id());
}

std::optional<cv::Mat> CameraControllerSiyiZT30::capture_color_image()
{
    cv::Mat frame;
    bool isGrabbed = capture_color_.grab();
    if (!isGrabbed) {
        RCLCPP_ERROR(this->get_logger(), "cannot grab frame from RTSP stream.");
        return std::nullopt;
    }
    capture_color_.retrieve(frame);
    return frame;
}

std::optional<cv::Mat> CameraControllerSiyiZT30::capture_ir_image()
{
    cv::Mat frame;
    bool isGrabbed = capture_ir_.grab();
    if (!isGrabbed) {
        RCLCPP_ERROR(this->get_logger(), "cannot grab frame from RTSP stream.");
        return std::nullopt;
    }
    capture_ir_.retrieve(frame);
    return frame;
}

bool CameraControllerSiyiZT30::center()
{
    return camera_.center();
}

bool CameraControllerSiyiZT30::set_angle(float azim, float elev)
{
    return static_cast<bool>(camera_.set_control_angle(azim, elev));
}

bool CameraControllerSiyiZT30::do_sweep(float pitch, float yaw_min, float yaw_max, float speed)
{
    if (sweeping_.load()) return true;
    
    sweeping_.store(true);
    sweep_thread_ = std::thread(&CameraControllerSiyiZT30::sweep_loop, this, pitch, yaw_min, yaw_max, speed);
    return true;
}

void CameraControllerSiyiZT30::stop_sweep()
{
    camera_.set_gimbal_rotation_speed(0, 0);
    if (!sweeping_.load()) return;
    sweeping_.store(false);
    if (sweep_thread_.joinable()) {
        sweep_thread_.join();
    }
    camera_.set_gimbal_rotation_speed(0, 0);
}

std::optional<float> CameraControllerSiyiZT30::get_distance()
{
    return camera_.get_cached_lrf_dist();
}

std::optional<std::tuple<float, float, float>> CameraControllerSiyiZT30::get_gimbal_attitude()
{
    auto res = camera_.get_cached_gimbal_attitude();
    if (res) {
        auto [yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel] = *res;
        return std::make_tuple(yaw, pitch, roll);
    } else {
        return std::nullopt;
    }
}

void CameraControllerSiyiZT30::sweep_loop(float pitch, float yaw_min, float yaw_max, float speed)
{
    bool dir_right = true;
    float yaw_ = 0.f, pitch_ = 0.f, roll_ = 0.f, yaw_vel_ = 0.f, pitch_vel_ = 0.f, roll_vel_ = 0.f;
    
    camera_.set_control_angle((yaw_min + yaw_max)/2, pitch);
    while (sweeping_.load()) // Wait for set_control_angle to finish
    {
        auto res = camera_.get_cached_gimbal_attitude();
        if (res) {
            std::tie(yaw_, pitch_, roll_, yaw_vel_, pitch_vel_, roll_vel_) = *res;
        }
        if (std::abs(yaw_vel_) < 1 && std::abs(pitch_vel_) < 1 && std::abs(roll_vel_) < 1) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    
    while (sweeping_.load())
    {
        auto res = camera_.get_cached_gimbal_attitude();
        if (res) {
            std::tie(yaw_, pitch_, roll_, yaw_vel_, pitch_vel_, roll_vel_) = *res;
        }
        if (dir_right && yaw_ < yaw_min) {
            dir_right = false;
        } else if (!dir_right && yaw_ > yaw_max) {
            dir_right = true;
        }
        float yaw_rate = dir_right ? speed : -speed;
        camera_.set_gimbal_rotation_speed(yaw_rate, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}