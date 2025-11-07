#include "auspex_cam/camera_controller_simIS.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

CameraControllerSimIS::CameraControllerSimIS(const std::string& platform_id,
                                             const std::string& ip_addr, int port,
                                             int stream_width_color, int stream_height_color, int stream_bitrate_color,
                                             int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                                             const float fps)
: CameraControllerBase(platform_id, "camera_controller_sim_is",
                       ip_addr, port,
                       stream_width_color, stream_height_color, stream_bitrate_color,
                       stream_width_ir,    stream_height_ir,    stream_bitrate_ir,
                       fps)
{
    // Derive vehicle id as in the old code: suffix after last '_'
    try {
        auto pos = platform_id.rfind('_');
        if (pos == std::string::npos || pos + 1 >= platform_id.size()) {
            throw std::runtime_error("platform_id format missing trailing _<id>");
        }
        vhcl_id_ = std::stoi(platform_id.substr(pos + 1));
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse vehicle id from platform_id='%s': %s. Defaulting to 0.",
                    platform_id.c_str(), e.what());
        vhcl_id_ = 0;
    }

    // Port & camera selection — same as old: base 5555 + vehicle_id, and Drone{vhcl_id+1}_front_30
    zmq_port_   = 5555 + vhcl_id_;
    camera_name_ = "/World/Drone" + std::to_string(vhcl_id_ + 1) + "_front_30";

    initCamera();
}

CameraControllerSimIS::~CameraControllerSimIS()
{
    shutdownCamera();
}

void CameraControllerSimIS::initCamera()
{
    try {
        // Connect and subscribe to all
        std::string endpoint = "tcp://localhost:" + std::to_string(zmq_port_);
        zmq_socket_.connect(endpoint);
        zmq_socket_.set(zmq::sockopt::subscribe, "");
        zmq_socket_.set(zmq::sockopt::rcvtimeo, 100); // 100 ms, same as old

        RCLCPP_INFO(this->get_logger(),
                    "Connected to ZMQ camera feed at %s for camera %s",
                    endpoint.c_str(), camera_name_.c_str());

        zmq_running_.store(true);
        zmq_thread_ = std::thread(&CameraControllerSimIS::zmq_receiver_loop, this);
    } catch (const zmq::error_t& e) {
        RCLCPP_ERROR(this->get_logger(), "ZMQ init failed: %s", e.what());
        zmq_running_.store(false);
    }
}

void CameraControllerSimIS::shutdownCamera()
{
    // Stop receiver thread
    zmq_running_.store(false);
    if (zmq_thread_.joinable()) {
        zmq_thread_.join();
    }

    // Close socket
    try {
        zmq_socket_.close();
        RCLCPP_INFO(this->get_logger(), "Closed ZMQ connection for vehicle %d", vhcl_id_ + 1);
    } catch (const zmq::error_t& e) {
        RCLCPP_WARN(this->get_logger(), "Error closing ZMQ socket: %s", e.what());
    }
}

bool CameraControllerSimIS::check_if_connected()
{
    // Consider "connected" if the receiver thread is running and at least one frame arrived.
    return zmq_running_.load() && frames_received_.load() > 0;
}

CameraControllerBase::Capabilities CameraControllerSimIS::available_capabilities() {
    return Capabilities{ true, false, false, false };
}

std::optional<cv::Mat> CameraControllerSimIS::capture_color_image()
{
    // Mirror old capture path: return latest available frame, BGR, if any
    if (!frame_available_.load()) {
        // Throttle at ~1 Hz
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "No frame available from ZMQ for camera %s", camera_name_.c_str());
        return std::nullopt;
    }

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (latest_frame_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Latest frame is empty for camera %s", camera_name_.c_str());
            return std::nullopt;
        }
        frame = latest_frame_.clone();
    }

    return frame; // BGR, CV_8UC3
}

void CameraControllerSimIS::zmq_receiver_loop()
{
    RCLCPP_INFO(this->get_logger(),
                "Starting ZMQ receiver loop for camera %s on port %d",
                camera_name_.c_str(), zmq_port_);

    while (zmq_running_.load()) {
        cv::Mat received;
        if (receive_zmq_frame(received)) {
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = std::move(received);
                frame_available_.store(true);
            }
            frames_received_.fetch_add(1, std::memory_order_relaxed);
        }

        std::this_thread::sleep_for(10ms); // prevent busy wait
    }

    RCLCPP_INFO(this->get_logger(),
                "ZMQ receiver loop stopped for camera %s",
                camera_name_.c_str());
}

bool CameraControllerSimIS::receive_zmq_frame(cv::Mat& frame)
{
    try {
        // 1) Receive metadata (non-blocking; rcvtimeo on socket governs total wait)
        zmq::message_t metadata_msg;
        auto res_meta = zmq_socket_.recv(metadata_msg, zmq::recv_flags::dontwait);
        if (!res_meta) {
            return false; // nothing right now
        }

        // Parse JSON metadata
        std::string metadata_str(static_cast<char*>(metadata_msg.data()), metadata_msg.size());
        nlohmann::json meta = nlohmann::json::parse(metadata_str);

        // Check camera name
        const std::string received_cam = meta.value<std::string>("camera_name", "");
        if (received_cam != camera_name_) {
            // Drain the image part even if it's not for us
            zmq::message_t discard;
            (void)zmq_socket_.recv(discard, zmq::recv_flags::none);
            return false;
        }

        // 2) Receive image payload
        zmq::message_t image_msg;
        auto res_img = zmq_socket_.recv(image_msg, zmq::recv_flags::none);
        if (!res_img) {
            RCLCPP_WARN(this->get_logger(), "Failed to receive image payload");
            return false;
        }

        // 3) Decode JPEG (old path supported only 'jpg')
        const std::string enc = meta.value<std::string>("encoding", "");
        if (enc != "jpg") {
            RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", enc.c_str());
            return false;
        }

        const uint8_t* ptr = static_cast<const uint8_t*>(image_msg.data());
        std::vector<uint8_t> buf(ptr, ptr + image_msg.size());
        cv::Mat decoded = cv::imdecode(buf, cv::IMREAD_COLOR); // BGR
        if (decoded.empty()) {
            RCLCPP_WARN(this->get_logger(), "imdecode returned empty frame");
            return false;
        }

        frame = std::move(decoded);
        return true;
    }
    catch (const zmq::error_t& e) {
        if (e.num() != EAGAIN) {
            RCLCPP_ERROR(this->get_logger(), "ZMQ receive error: %s", e.what());
        }
        return false;
    }
    catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in ZMQ frame reception: %s", e.what());
        return false;
    }
}
