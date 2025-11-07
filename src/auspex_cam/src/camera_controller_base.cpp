#include "auspex_cam/camera_controller_base.hpp"

CameraControllerBase::CameraControllerBase(const std::string& platform_id,
                                           const std::string& node_name,
                                           const std::string& ip_addr,
                                           const int port,
                                           int stream_width_color,
                                           int stream_height_color,
                                           int stream_bitrate_color,
                                           int stream_width_ir,
                                           int stream_height_ir,
                                           int stream_bitrate_ir,
                                           const float fps)
    : Node(platform_id + "_" + node_name),
    fps_(fps),
    platform_id_(platform_id),
    ip_addr_(ip_addr),
    port_(port),
    stream_width_color_(stream_width_color),
    stream_height_color_(stream_height_color),
    stream_bitrate_color_(stream_bitrate_color),
    stream_width_ir_(stream_width_ir),
    stream_height_ir_(stream_height_ir),
    stream_bitrate_ir_(stream_bitrate_ir)
{
    // nothing to do here, call init()
}

CameraControllerBase::~CameraControllerBase()
{
    stop_capture();
}

void CameraControllerBase::init()
{
    Capabilities caps = available_capabilities();
    if (caps.image_color || caps.image_infrared)
    {
        
        // Connection Checking
        connection_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&CameraControllerBase::on_connection_timer, this)
        );
        
        // Image Publishing
        capture_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
            std::bind(&CameraControllerBase::on_capture_timer, this)
        );
        capture_timer_->cancel();
        
        // CameraControllerStartCapture
        srv_start_capture_ = create_service<auspex_msgs::srv::CameraControllerStartCapture>(
            platform_id_ + std::string("/camera_controller_start_capture"),
            std::bind(&CameraControllerBase::on_srv_start_capture, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // CameraControllerStopCapture
        srv_stop_capture_ = create_service<auspex_msgs::srv::CameraControllerStopCapture>(
            platform_id_ + std::string("/camera_controller_stop_capture"),
            std::bind(&CameraControllerBase::on_srv_stop_capture, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
    
    if (caps.image_color)
    {
        stream_color_ = std::make_unique<StreamHandler>(
            ip_addr_,
            port_,
            (platform_id_ + "/stream/color"),
            stream_width_color_,
            stream_height_color_,
            fps_,
            stream_bitrate_color_
        );
        ip_addr_ = stream_color_->get_ip();
        port_ = stream_color_->get_port();
    }
    
    if (caps.image_infrared) {
        stream_ir_ = std::make_unique<StreamHandler>(
            ip_addr_,
            port_,
            (platform_id_ + "/stream/ir"),
            stream_width_ir_,
            stream_height_ir_,
            fps_,
            stream_bitrate_ir_
        );
        ip_addr_ = stream_ir_->get_ip();
        port_ = stream_ir_->get_port();
    }
    
    if (caps.gimbal)
    {
        // CameraGimbalCenter
        srv_center_ = create_service<auspex_msgs::srv::CameraGimbalCenter>(
            platform_id_ + std::string("/camera_gimbal_center"),
            std::bind(&CameraControllerBase::on_srv_center, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // CameraGimbalSetAngle
        srv_set_angle_ = create_service<auspex_msgs::srv::CameraGimbalSetAngle>(
            platform_id_ + std::string("/camera_gimbal_set_angle"),
            std::bind(&CameraControllerBase::on_srv_set_angle, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // CameraGimbalStartSweep
        srv_start_sweep_ = create_service<auspex_msgs::srv::CameraGimbalStartSweep>(
            platform_id_ + std::string("/camera_gimbal_start_sweep"),
            std::bind(&CameraControllerBase::on_srv_start_sweep, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // CameraGimbalStopSweep
        srv_stop_sweep_ = create_service<auspex_msgs::srv::CameraGimbalStopSweep>(
            platform_id_ + std::string("/camera_gimbal_stop_sweep"),
            std::bind(&CameraControllerBase::on_srv_stop_sweep, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
}


////////////////////////////
// Capture and Publishing //
////////////////////////////

void CameraControllerBase::start_capture() {
    if (!is_capturing()) {
        if (stream_color_) stream_color_->start();
        if (stream_ir_) stream_ir_->start();
        if (capture_timer_) capture_timer_->reset();
        
        RCLCPP_INFO(this->get_logger(), "Camera started capturing.");
        if (stream_color_ || stream_ir_) RCLCPP_INFO(this->get_logger(), "RTSP stream URL(s):");
        if (stream_color_) RCLCPP_INFO(this->get_logger(), "rtsp://%s:%d/%s", stream_color_->get_ip().c_str(), stream_color_->get_port(), stream_color_->get_mount().c_str());
        if (stream_ir_) RCLCPP_INFO(this->get_logger(), "rtsp://%s:%d/%s", stream_ir_->get_ip().c_str(), stream_ir_->get_port(), stream_ir_->get_mount().c_str());
    }
}

void CameraControllerBase::stop_capture() {
    if (is_capturing()) {
        if (capture_timer_) capture_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Camera stopped capturing.");
        
        if (stream_color_) stream_color_->stop();
        if (stream_ir_) stream_ir_->stop();
    }
}

bool CameraControllerBase::is_capturing() const {
    if (!capture_timer_) return false;
    return !capture_timer_->is_canceled();
}

void CameraControllerBase::capture_frame()
{
    Capabilities caps = available_capabilities();
    
    // Build Framedata
    // auto msg = TmpFramedata();
    // msg.platform_id = platform_id_;
    // msg.team_id = "drone_team";
    // msg.fps = fps_;
    
    if (caps.image_color) {
        if (auto res = capture_color_image()) {
            auto image = *res;
            stream_color_->submit_frame(image);
        }
    }
    if (caps.image_infrared) {
        if (auto res = capture_ir_image()) {
            auto image = *res;
            stream_ir_->submit_frame(image);
        }
    }
    // if (caps.range_finder) {
    //     auto res = get_distance();
    //     msg.distance = res.value_or(0.f);
    // }
    // if (caps.gimbal) {
    //     auto res = get_gimbal_attitude();
    //     if (res) {
    //         // auto [yaw, pitch, roll] = *res;
    //         // ToDo
    //     }
    // }
    // frame_publisher_->publish(msg);
}

void CameraControllerBase::on_capture_timer() {
    if (capture_mutex_.try_lock()) {
        capture_frame();
        capture_mutex_.unlock();
    }
    // If mutex is locked, return immediately without blocking
}

void CameraControllerBase::on_connection_timer()
{
    auto status = check_if_connected();
    if (is_connected != status) {
        if (status) {
            RCLCPP_INFO(this->get_logger(), "Camera went online.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Camera went offline.");
        }
    }
    is_connected = status;
}

////////////////////////////////////
// Messages, services and actions //
////////////////////////////////////

void CameraControllerBase::on_srv_start_capture(
    const std::shared_ptr<auspex_msgs::srv::CameraControllerStartCapture::Request>,
    std::shared_ptr<auspex_msgs::srv::CameraControllerStartCapture::Response> res)
{
    start_capture();
    res->success = true;
}

void CameraControllerBase::on_srv_stop_capture(
    const std::shared_ptr<auspex_msgs::srv::CameraControllerStopCapture::Request>,
    std::shared_ptr<auspex_msgs::srv::CameraControllerStopCapture::Response> res)
{
    stop_capture();
    res->success = true;
}

void CameraControllerBase::on_srv_center(
    const std::shared_ptr<auspex_msgs::srv::CameraGimbalCenter::Request>,
    std::shared_ptr<auspex_msgs::srv::CameraGimbalCenter::Response> res)
{
    stop_sweep();
    res->success = center();
}

void CameraControllerBase::on_srv_set_angle(
    const std::shared_ptr<auspex_msgs::srv::CameraGimbalSetAngle::Request> req,
    std::shared_ptr<auspex_msgs::srv::CameraGimbalSetAngle::Response> res)
{
    stop_sweep();
    res->success = set_angle(req->azimuth, req->elevation);
}

void CameraControllerBase::on_srv_start_sweep(
    const std::shared_ptr<auspex_msgs::srv::CameraGimbalStartSweep::Request> req,
    std::shared_ptr<auspex_msgs::srv::CameraGimbalStartSweep::Response> res)
{
    res->success = do_sweep(req->pitch, req->yaw_min, req->yaw_max, req->speed);
}

void CameraControllerBase::on_srv_stop_sweep(
    const std::shared_ptr<auspex_msgs::srv::CameraGimbalStopSweep::Request>,
    std::shared_ptr<auspex_msgs::srv::CameraGimbalStopSweep::Response> res)
{
    stop_sweep();
    res->success = true;
}

//////////////////////
// Helper Functions //
//////////////////////

std::vector<uint8_t> CameraControllerBase::compress_image_to_jpg(cv::Mat image, int quality)
{
    std::vector<int> compression_params = {
        cv::IMWRITE_JPEG_QUALITY, quality
    };
    std::vector<uint8_t> jpg_data;
    if (!cv::imencode(".jpg", image, jpg_data, compression_params)) {
        RCLCPP_ERROR(get_logger(), "JPG encoding failed");
        return {};
    }
    return jpg_data;
}

///////////////////////////
// Placeholder functions //
///////////////////////////

CameraControllerBase::Capabilities CameraControllerBase::available_capabilities()
{
    Capabilities cap{
        false, // image_color
        false, // image_infrared
        false, // range_finder
        false, // gimbal
    };
    return cap;
}

bool CameraControllerBase::check_if_connected()
{
    RCLCPP_WARN(this->get_logger(), "Please override the 'check_if_connected' function.");
    return false;
}

std::optional<cv::Mat> CameraControllerBase::capture_color_image()
{
    RCLCPP_WARN(this->get_logger(), "CameraController lacks capability 'capture_color_image'.");
    return std::nullopt;
}

std::optional<cv::Mat> CameraControllerBase::capture_ir_image()
{
    RCLCPP_WARN(this->get_logger(), "CameraController lacks capability 'capture_ir_image'.");
    return std::nullopt;
}

bool CameraControllerBase::center()
{
    RCLCPP_WARN(this->get_logger(), "CameraController lacks capability 'center'.");
    return false;
}

bool CameraControllerBase::set_angle(float /*azim*/, float /*elev*/)
{
    RCLCPP_WARN(this->get_logger(), "CameraController lacks capability 'set_angle'.");
    return false;
}

bool CameraControllerBase::do_sweep(float /*pitch*/, float /*yaw_min*/, float /*yaw_max*/, float /*speed*/)
{
    RCLCPP_WARN(this->get_logger(), "ToDo 'sweep'.");
    return false;
}

void CameraControllerBase::stop_sweep()
{
    return;
}

std::optional<float> CameraControllerBase::get_distance()
{
    RCLCPP_WARN(this->get_logger(), "CameraController lacks capability 'get_distance'.");
    return std::nullopt;
}

std::optional<std::tuple<float, float, float>> CameraControllerBase::get_gimbal_attitude()
{
    RCLCPP_WARN(this->get_logger(), "CameraController lacks capability 'get_gimbal_attitude'.");
    return std::nullopt;
}