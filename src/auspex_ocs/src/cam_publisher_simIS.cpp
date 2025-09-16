#include "auspex_ocs/cam_publisher_simIS.hpp"
#include <chrono>
#include <iostream>

SimCamPublisherIS::SimCamPublisherIS(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps)
    : CamPublisherBase(platform_id, "sim_cam_publisher_is", transmitHeight, transmitWidth, fps),
      zmq_context_(1),
      zmq_socket_(zmq_context_, ZMQ_SUB),
      zmq_running_(false),
      frame_available_(false)
{
    vhcl_id_ = std::stoi(platform_id.substr(platform_id.rfind('_')+1));
    
    // Calculate ZMQ port: base port 5555 + vehicle_id for front_30 camera
    zmq_port_ = 5555 + vhcl_id_;
    camera_name_ = "/World/Drone" + std::to_string(vhcl_id_ + 1) + "_front_30";
    
    // Configure ZMQ socket
    try {
        std::string zmq_endpoint = "tcp://localhost:" + std::to_string(zmq_port_);
        zmq_socket_.connect(zmq_endpoint);
        zmq_socket_.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
        zmq_socket_.set(zmq::sockopt::rcvtimeo, 100);  // 100ms timeout
        
        RCLCPP_INFO(get_logger(), "Connected to ZMQ camera feed at %s for camera %s", zmq_endpoint.c_str(), camera_name_.c_str());
        
        // Start ZMQ receiver thread
        zmq_running_ = true;
        zmq_thread_ = std::thread(&SimCamPublisherIS::zmq_receiver_loop, this);
        
    } catch (const zmq::error_t& e) {
        RCLCPP_ERROR(get_logger(), "ZMQ initialization failed: %s", e.what());
        zmq_running_ = false;
    }
}

SimCamPublisherIS::~SimCamPublisherIS() {
    stopCapture();
    
    // Stop ZMQ receiver thread
    zmq_running_ = false;
    if (zmq_thread_.joinable()) {
        zmq_thread_.join();
    }
    
    // Close ZMQ socket
    try {
        zmq_socket_.close();
        RCLCPP_INFO(get_logger(), "Closed ZMQ connection for vehicle %d", vhcl_id_ + 1);
    } catch (const zmq::error_t& e) {
        RCLCPP_WARN(get_logger(), "Error closing ZMQ socket: %s", e.what());
    }
}

void SimCamPublisherIS::zmq_receiver_loop() {
    RCLCPP_INFO(get_logger(), "Starting ZMQ receiver loop for camera %s on port %d", camera_name_.c_str(), zmq_port_);
    
    while (zmq_running_) {
        cv::Mat received_frame;
        if (receive_zmq_frame(received_frame)) {
            // Thread-safe frame update
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = received_frame.clone();
                frame_available_ = true;
            }
        }
        
        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_INFO(get_logger(), "ZMQ receiver loop stopped for camera %s", camera_name_.c_str());
}

bool SimCamPublisherIS::receive_zmq_frame(cv::Mat& frame) {
    try {
        // Receive metadata
        zmq::message_t metadata_msg;
        auto result = zmq_socket_.recv(metadata_msg, zmq::recv_flags::dontwait);
        if (!result) {
            return false; // No message available
        }
        
        // Parse metadata JSON
        std::string metadata_str(static_cast<char*>(metadata_msg.data()), metadata_msg.size());
        nlohmann::json metadata = nlohmann::json::parse(metadata_str);
        
        // Check if this is the camera we want
        std::string received_camera = metadata.value("camera_name", "");
        if (received_camera != camera_name_) {
            // Receive and discard the image data
            zmq::message_t image_msg;
            auto discard_result = zmq_socket_.recv(image_msg, zmq::recv_flags::none);
            if (!discard_result) {
                RCLCPP_WARN(get_logger(), "Failed to receive image data for discarding");
            }
            return false;
        }
        
        // Receive image data
        zmq::message_t image_msg;
        result = zmq_socket_.recv(image_msg, zmq::recv_flags::none);
        if (!result) {
            RCLCPP_WARN(get_logger(), "Failed to receive image data");
            return false;
        }
        
        // Decode JPEG image
        std::string encoding = metadata.value("encoding", "");
        if (encoding == "jpg") {
            std::vector<uint8_t> image_data(
                static_cast<uint8_t*>(image_msg.data()),
                static_cast<uint8_t*>(image_msg.data()) + image_msg.size()
            );
            
            frame = cv::imdecode(image_data, cv::IMREAD_COLOR);
            return true;
            
        } else {
            RCLCPP_WARN(get_logger(), "Unsupported image encoding: %s", encoding.c_str());
            return false;
        }
        
    } catch (const zmq::error_t& e) {
        if (e.num() != EAGAIN) { // EAGAIN is expected for timeouts
            RCLCPP_ERROR(get_logger(), "ZMQ receive error: %s", e.what());
        }
        return false;
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(get_logger(), "JSON parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in ZMQ frame reception: %s", e.what());
        return false;
    }
}

void SimCamPublisherIS::captureFrame() {
    // Use instance-level mutex to prevent multiple simultaneous captures from the same camera
    std::lock_guard<std::mutex> lock(capture_mutex_);

    // Check if we have a frame available
    if (!frame_available_.load()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No frame available from ZMQ for camera %s", camera_name_.c_str());
        return;
    }

    cv::Mat current_frame;
    
    // Thread-safe frame retrieval
    {
        std::lock_guard<std::mutex> frame_lock(frame_mutex_);
        if (latest_frame_.empty()) {
            RCLCPP_WARN(get_logger(), "Latest frame is empty for camera %s", camera_name_.c_str());
            return;
        }
        current_frame = latest_frame_.clone();
    }
    
    // Process and publish image
    cv::Mat rgba;
    cv::cvtColor(current_frame, rgba, cv::COLOR_RGB2RGBA);

    cv::resize(rgba, rgba, cv::Size(transmitWidth_, transmitHeight_), 0, 0, cv::INTER_LINEAR);

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);
    std::vector<uint8_t> jpg;
    if (!cv::imencode(".jpg", rgba, jpg, compression_params)) {
        RCLCPP_ERROR(get_logger(), "JPG encoding failed");
        return;
    }

    FrameData image_msg;
    image_msg.platform_id = platform_id_;
    image_msg.team_id = "drone_team";
    image_msg.image_compressed.data = std::move(jpg);
    image_msg.image_compressed.header.stamp = get_clock()->now();
    image_msg.image_compressed.format = "jpeg";
    image_msg.fps = static_cast<int>(fps_);
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