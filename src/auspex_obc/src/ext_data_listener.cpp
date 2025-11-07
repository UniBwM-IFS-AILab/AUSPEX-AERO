#include "auspex_obc/ext_data_listener.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ExternalDataListener::ExternalDataListener(const std::string& own_platform_id)
    : Node(own_platform_id + "_external_data_listener"), own_platform_id_(own_platform_id) {
    
    // Set up QoS profile for sensor data (same as DroneStatePublisher)
    rmw_qos_profile_t sensor_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(
        rclcpp::QoSInitialization(sensor_profile.history, sensor_profile.depth),
        sensor_profile
    );
    
    // Create subscription to platform_state topic
    subscription_ = this->create_subscription<PlatformState>(
        "/platform_state",
        sensor_qos,
        std::bind(&ExternalDataListener::platformStateCallback, this, std::placeholders::_1)
    );
    
    // Set up timer for periodic cleanup of outdated platforms (every 30 seconds)
    cleanup_timer_ = this->create_wall_timer(
        std::chrono::seconds(30),
        [this]() { 
            size_t removed = this->removeOutdatedPlatforms();
            if (removed > 0) {
                RCLCPP_INFO(this->get_logger(), "Removed %zu outdated platform(s)", removed);
            }
        }
    );
    
    RCLCPP_INFO(this->get_logger(), "ExternalDataListener initialized for platform: %s", own_platform_id_.c_str());
}

void ExternalDataListener::platformStateCallback(const PlatformState::SharedPtr msg) {
    // Skip our own platform messages
    if (msg->platform_id == own_platform_id_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    
    // Create or update platform data
    auto platform_data = std::make_shared<PlatformData>();
    platform_data->platform_id = msg->platform_id;
    platform_data->team_id = msg->team_id;
    platform_data->platform_status = msg->platform_status;
    platform_data->platform_pose = msg->platform_pose;
    platform_data->platform_gps_position = msg->platform_gps_position;
    platform_data->sensor_id = msg->sensor_id;
    platform_data->sensor_position = msg->sensor_position;
    platform_data->gimbal_orientation = msg->gimbal_orientation;
    platform_data->last_update = this->now();
    
    // Store/update the platform data
    platforms_[msg->platform_id] = platform_data;
}

std::vector<geographic_msgs::msg::GeoPoint> ExternalDataListener::getAllPlatformPositions() const {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    std::vector<geographic_msgs::msg::GeoPoint> positions;
    positions.reserve(platforms_.size());
    
    for (const auto& [platform_id, platform_data] : platforms_) {
        positions.push_back(platform_data->platform_gps_position);
    }
    
    return positions;
}

std::vector<geometry_msgs::msg::Pose> ExternalDataListener::getAllPlatformPoses() const {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(platforms_.size());
    
    for (const auto& [platform_id, platform_data] : platforms_) {
        poses.push_back(platform_data->platform_pose);
    }
    
    return poses;
}

std::vector<geometry_msgs::msg::Point> ExternalDataListener::getAllPlatformGpsPositions() const {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    std::vector<geometry_msgs::msg::Point> gps_positions;
    gps_positions.reserve(platforms_.size());
    
    for (const auto& [platform_id, platform_data] : platforms_) {
        // Convert GeoPoint to Point for backward compatibility
        geometry_msgs::msg::Point point;
        point.x = platform_data->platform_gps_position.longitude;
        point.y = platform_data->platform_gps_position.latitude;
        point.z = platform_data->platform_gps_position.altitude;
        gps_positions.push_back(point);
    }
    
    return gps_positions;
}

std::vector<std::string> ExternalDataListener::getAllPlatformIds() const {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    std::vector<std::string> platform_ids;
    platform_ids.reserve(platforms_.size());
    
    for (const auto& [platform_id, platform_data] : platforms_) {
        platform_ids.push_back(platform_id);
    }
    
    return platform_ids;
}

std::shared_ptr<const PlatformData> ExternalDataListener::getPlatformData(const std::string& platform_id) const {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    auto it = platforms_.find(platform_id);
    if (it != platforms_.end()) {
        return it->second;
    }
    return nullptr;
}

size_t ExternalDataListener::getPlatformCount() const {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    return platforms_.size();
}

bool ExternalDataListener::hasPlatform(const std::string& platform_id) const {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    return platforms_.find(platform_id) != platforms_.end();
}

std::vector<std::string> ExternalDataListener::getPlatformsWithinDistance(
    const geographic_msgs::msg::GeoPoint& reference_pose, double max_distance) const {
    
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    std::vector<std::string> nearby_platforms;
    
    for (const auto& [platform_id, platform_data] : platforms_) {
        double distance = calculateDistance(reference_pose, platform_data->platform_gps_position);
        if (distance <= max_distance) {
            nearby_platforms.push_back(platform_id);
        }
    }
    
    return nearby_platforms;
}

size_t ExternalDataListener::removeOutdatedPlatforms(double timeout_seconds) {
    std::lock_guard<std::mutex> lock(platforms_mutex_);
    auto current_time = this->now();
    size_t removed_count = 0;
    
    auto it = platforms_.begin();
    while (it != platforms_.end()) {
        auto time_diff = current_time - it->second->last_update;
        if (time_diff.seconds() > timeout_seconds) {
            RCLCPP_DEBUG(this->get_logger(), "Removing outdated platform: %s", it->first.c_str());
            it = platforms_.erase(it);
            removed_count++;
        } else {
            ++it;
        }
    }
    
    return removed_count;
}

double ExternalDataListener::calculateDistance(
    const geographic_msgs::msg::GeoPoint& point1, const geographic_msgs::msg::GeoPoint& point2) const {
    
    // Use Haversine formula for geographic distance calculation
    const double R = 6371000.0; // Earth's radius in meters
    
    // Convert degrees to radians
    double lat1_rad = point1.latitude * M_PI / 180.0;
    double lat2_rad = point2.latitude * M_PI / 180.0;
    double dlat_rad = (point2.latitude - point1.latitude) * M_PI / 180.0;
    double dlon_rad = (point2.longitude - point1.longitude) * M_PI / 180.0;
    
    // Haversine formula for great circle distance
    double a = std::sin(dlat_rad/2) * std::sin(dlat_rad/2) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(dlon_rad/2) * std::sin(dlon_rad/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    double horizontal_distance = R * c;
    
    // Add altitude difference
    double alt_diff = point2.altitude - point1.altitude;
    
    // Calculate 3D distance using Pythagorean theorem
    return std::sqrt(horizontal_distance * horizontal_distance + alt_diff * alt_diff);
}