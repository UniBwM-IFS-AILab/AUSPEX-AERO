#include "auspex_ocs/cam_publisher_simUE.hpp"

SimCamPublisherUE::SimCamPublisherUE(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps)
    : CamPublisherBase(platform_id, "sim_cam_publisher_ue", transmitHeight, transmitWidth, fps)
{
    vhcl_id_ = std::stoi(platform_id.substr(platform_id.rfind('_')+1));
    
    try {
        // Get shared client through connection manager
        auto client = AirSimConnectionManager::getClient();
        RCLCPP_INFO(get_logger(), "SimCamPublisherUE initialized for vehicle %d using shared connection", vhcl_id_+1);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize SimCamPublisherUE for vehicle %d: %s", vhcl_id_+1, e.what());
        throw;
    }
}

SimCamPublisherUE::~SimCamPublisherUE() {
    stopCapture();
    
    // Release our reference to the shared client
    try {
        AirSimConnectionManager::releaseClient();
        RCLCPP_INFO(get_logger(), "Released connection reference for vehicle %d", vhcl_id_+1);
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Error releasing connection reference: %s", e.what());
    }
}

void SimCamPublisherUE::captureFrame() {
    // Use instance-level mutex to prevent multiple simultaneous captures from the same camera
    std::lock_guard<std::mutex> lock(capture_mutex_);
    
    try {
        // Get the shared client
        auto sim_client = AirSimConnectionManager::getClient();
        if (!sim_client) {
            RCLCPP_ERROR(get_logger(), "Failed to get AirSim client connection");
            return;
        }
        
        // Build image request
        std::vector<msr::airlib::ImageCaptureBase::ImageRequest> reqs = {
            {"front_30", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false}
        };

        cv::Mat img;
        int w = 0;
        int h = 0;

        std::string vehicle_name = "Drone" + std::to_string(vhcl_id_+1);
        auto responses = sim_client->simGetImages(reqs, vehicle_name);
        
        if (responses.empty()) {
            RCLCPP_ERROR(get_logger(), "No image responses received for vehicle '%s'", vehicle_name.c_str());
            return;
        }
        
        auto& raw = responses[0].image_data_uint8;
        if (raw.empty()) {
            RCLCPP_ERROR(get_logger(), "Colosseum returned an empty image for vehicle '%s'", vehicle_name.c_str());
            return;
        }
        
        w = responses[0].width;
        h = responses[0].height;
        img = cv::Mat(h, w, CV_8UC3, raw.data());
        
        // Process and publish image
        cv::Mat rgba;
        cv::cvtColor(img, rgba, cv::COLOR_RGB2RGBA);

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
        
    } catch (const std::exception& e) {
        std::string vehicle_name = "Drone" + std::to_string(vhcl_id_+1);
        RCLCPP_ERROR(get_logger(), "simGetImages RPC failed for vehicle '%s': %s", vehicle_name.c_str(), e.what());
        
        // Try to reconnect on next call if there was a connection error
        if (std::string(e.what()).find("connection") != std::string::npos ||
            std::string(e.what()).find("Connection") != std::string::npos) {
            AirSimConnectionManager::forceDisconnect();
        }
    }
}