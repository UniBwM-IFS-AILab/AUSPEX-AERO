#include "auspex_cam/camera_controller_simUE.hpp"
#include "airsim_connection_manager/airsim_connection_manager.hpp"

using msr::airlib::ImageCaptureBase;

CameraControllerSimUE::CameraControllerSimUE(const std::string& platform_id,
                                             const std::string& ip_addr, int port,
                                             int stream_width_color, int stream_height_color, int stream_bitrate_color,
                                             int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                                             const float fps)
    : CameraControllerBase(platform_id, "camera_controller_sim_ue",
                           ip_addr, port,
                           stream_width_color, stream_height_color, stream_bitrate_color,
                           stream_width_ir,    stream_height_ir,    stream_bitrate_ir,
                           fps)
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

CameraControllerSimUE::~CameraControllerSimUE()
{
    stop_capture();
    
    // Release our reference to the shared client
    try {
        AirSimConnectionManager::releaseClient();
        RCLCPP_INFO(get_logger(), "Released connection reference for vehicle %d", vhcl_id_+1);
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Error releasing connection reference: %s", e.what());
    }
}

bool CameraControllerSimUE::check_if_connected()
{
    try {
        auto client = AirSimConnectionManager::getClient();
        if (!client) return false;
        return true;
    } catch (...) {
        return false;
    }
}

CameraControllerBase::Capabilities CameraControllerSimUE::available_capabilities() {
    return Capabilities{ true, false, false, false };
}

std::optional<cv::Mat> CameraControllerSimUE::capture_color_image()
{
    // Use instance-level mutex to prevent multiple simultaneous captures from the same camera
    std::lock_guard<std::mutex> lock(capture_mutex_);
    
    try {
        // Get the shared client
        auto sim_client = AirSimConnectionManager::getClient();
        if (!sim_client) {
            RCLCPP_ERROR(get_logger(), "Failed to get AirSim client connection");
            return std::nullopt;
        }
        
        // Build image request
        std::vector<msr::airlib::ImageCaptureBase::ImageRequest> reqs = {
            {"front_30", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false}
        };

        std::string vehicle_name = "Drone" + std::to_string(vhcl_id_+1);
        auto responses = sim_client->simGetImages(reqs, vehicle_name);
        
        if (responses.empty()) {
            RCLCPP_ERROR(get_logger(), "No image responses received for vehicle '%s'", vehicle_name.c_str());
            return std::nullopt;
        }
        
        auto& raw = responses[0].image_data_uint8;
        if (raw.empty()) {
            RCLCPP_ERROR(get_logger(), "Colosseum returned an empty image for vehicle '%s'", vehicle_name.c_str());
            return std::nullopt;
        }
        
        int w = responses[0].width;
        int h = responses[0].height;
        cv::Mat img(h, w, CV_8UC3);
        std::memcpy(img.data, raw.data(), raw.size());
        return img;
        
        // // Convert RGB to RGBA for consistent processing
        // cv::Mat rgba;
        // cv::cvtColor(img, rgba, cv::COLOR_RGB2RGBA);

        // // Stream frame via RTSP and publish metadata via ROS
        // streamFrameAndPublishMetadata(rgba);
        
    } catch (const std::exception& e) {
        std::string vehicle_name = "Drone" + std::to_string(vhcl_id_+1);
        RCLCPP_ERROR(get_logger(), "simGetImages RPC failed for vehicle '%s': %s", vehicle_name.c_str(), e.what());
        
        // Try to reconnect on next call if there was a connection error
        if (std::string(e.what()).find("connection") != std::string::npos ||
            std::string(e.what()).find("Connection") != std::string::npos) {
            AirSimConnectionManager::forceDisconnect();
        }
        return std::nullopt;
    }
}
