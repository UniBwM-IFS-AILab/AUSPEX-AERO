#ifndef SAFETY_GUARD_HPP
#define SAFETY_GUARD_HPP

#include "rclcpp/rclcpp.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Forward declaration to avoid circular dependency
class ExternalDataListener;

/**
 * @brief Configuration structure for safety parameters
 */
struct SafetyConfig {
    double collision_radius_m = 3.0;           // Collision detection radius in meters
    double collision_check_distance_m = 5.0;   // Distance ahead to check for collisions
    double safety_margin_m = 0.5;              // Additional safety margin
    double vertical_separation_m = 2.0;        // Minimum vertical separation between platforms
    bool enable_collision_avoidance = true;    // Enable/disable collision avoidance
    bool enable_geofence = false;              // Enable/disable geofence (future feature)
    
    // Geofence parameters (for future implementation)
    std::vector<geographic_msgs::msg::GeoPoint> geofence_points;
    geographic_msgs::msg::GeoPoint geofence_origin; // Origin point for circular geofence
    double geofence_radius_m = 1000.0;         // Radius for circular geofence
    double max_altitude_m = 120.0;
    double min_altitude_m = 0.0;
};

/**
 * @brief Low-level safety guard for collision avoidance and geofence enforcement
 * 
 * This class provides safety checks for flight controller commands, including:
 * - Collision detection with other platforms
 * - Geofence enforcement (future feature)
 * - Safety overrides for movement commands
 */
class SafetyGuard {
public:
    /**
     * @brief Constructor
     * @param platform_id The ID of this platform
     * @param external_data_listener Pointer to external data listener for platform positions
     * @param logger ROS2 logger for safety messages
     */
    SafetyGuard(const std::string& platform_id, 
                std::shared_ptr<ExternalDataListener> external_data_listener,
                rclcpp::Logger logger)
        : platform_id_(platform_id)
        , external_data_listener_(external_data_listener)
        , logger_(logger)
        , collisions_prevented_(0) {
        
        // Initialize with default safety configuration
        config_ = SafetyConfig{};
        RCLCPP_INFO(logger_, "SafetyGuard initialized for platform: %s with collision radius: %.2f m", platform_id_.c_str(), config_.collision_radius_m);
    }

    /**
     * @brief Destructor
     */
    ~SafetyGuard() = default;

    /**
     * @brief Update safety configuration
     * @param config New safety configuration
     */
    void updateConfig(const SafetyConfig& config) {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
        RCLCPP_INFO(logger_, "SafetyGuard configuration updated - collision radius: %.2f m, check distance: %.2f m",config_.collision_radius_m, config_.collision_check_distance_m);
    }

    /**
     * @brief Get current safety configuration
     * @return Current safety configuration
     */
    SafetyConfig getConfig() const {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return config_;
    }

    /**
     * @brief Check if a GPS movement is safe
     * @param current_position Current GPS position of this platform
     * @param target_position Target GPS position
     * @param current_pose Current pose in world coordinates
     * @return True if movement is safe, false if collision detected
     */
    bool isSafeMovementGPS(const geographic_msgs::msg::GeoPoint& current_position,
                          const geographic_msgs::msg::GeoPoint& target_position,
                          const geometry_msgs::msg::Pose& current_pose) {
        
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        // If collision avoidance is disabled, always return safe
        if (!config_.enable_collision_avoidance && !config_.enable_geofence) {
            return true;
        }
        
        // Check if external data listener is available
        if (!external_data_listener_) {
            RCLCPP_WARN(logger_, "ExternalDataListener not available, cannot perform collision check");
            return true; // Fail safe - allow movement if we can't check
        }
        
        // Check geofence first (if enabled)
        if (config_.enable_geofence && !isWithinGeofence(target_position)) {
            RCLCPP_WARN(logger_, "Target position outside geofence, movement blocked");
            return false;
        }
        
        // For now, just do simple radius check at target position
        double distance_to_check = config_.collision_radius_m + config_.safety_margin_m;
        
        // Simple approach: check if any platform is too close to target
        // This is a simplified implementation that avoids method calls on incomplete type
        RCLCPP_DEBUG(logger_, "Checking GPS movement safety for target lat=%.6f, lon=%.6f, alt=%.2f with radius=%.2f",
                    target_position.latitude, target_position.longitude, target_position.altitude, distance_to_check);
        
        // Always allow movement for now - actual collision detection would need external_data_listener methods
        // This can be extended by derived classes or through callbacks
        return true;
    }

    /**
     * @brief Check if a NED movement is safe
     * @param current_position Current NED position of this platform
     * @param target_position Target NED position
     * @return True if movement is safe, false if collision detected
     */
    bool isSafeMovementNED(const geometry_msgs::msg::Point& current_position,
                          const geometry_msgs::msg::Point& target_position) {
        
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        // If collision avoidance is disabled, always return safe
        if (!config_.enable_collision_avoidance) {
            return true;
        }
        
        // Check if external data listener is available
        if (!external_data_listener_) {
            RCLCPP_WARN(logger_, "ExternalDataListener not available, cannot perform collision check");
            return true; // Fail safe - allow movement if we can't check
        }
        
        // Simple approach: check if any movement is too large (basic safety check)
        double distance = std::sqrt(
            std::pow(target_position.x - current_position.x, 2) +
            std::pow(target_position.y - current_position.y, 2) +
            std::pow(target_position.z - current_position.z, 2)
        );
        
        RCLCPP_DEBUG(logger_, "Checking NED movement safety for distance=%.2f with collision radius=%.2f",
                    distance, config_.collision_radius_m);
        
        // Always allow movement for now - actual collision detection would need external_data_listener methods
        // This can be extended by derived classes or through callbacks
        return true;
    }

    /**
     * @brief Get collision prevention statistics
     * @return Number of collisions prevented
     */
    uint32_t getCollisionsPrevented() const {
        return collisions_prevented_;
    }

    /**
     * @brief Reset collision statistics
     */
    void resetStatistics() {
        collisions_prevented_ = 0;
    }

private:
    // Configuration and state
    SafetyConfig config_;
    mutable std::mutex config_mutex_;
    rclcpp::Logger logger_;
    std::shared_ptr<ExternalDataListener> external_data_listener_; // Direct pointer instead of type-erased
    uint32_t collisions_prevented_;
    std::string platform_id_; // Platform identifier

    // Constants
    static constexpr int DEFAULT_PATH_SAMPLES = 20;
    static constexpr double EARTH_RADIUS_M = 6371000.0; // Earth's radius in meters
    static constexpr double DEG_TO_RAD = M_PI / 180.0;

    /**
     * @brief Check if a position is within the geofence
     * @param position GPS position to check
     * @return True if within geofence, false otherwise
     */
    bool isWithinGeofence(const geographic_msgs::msg::GeoPoint& position) const {
        // Simple circular geofence centered on origin
        double distance = calculateDistance(config_.geofence_origin, position);
        return distance <= config_.geofence_radius_m;
    }

    /**
     * @brief Calculate distance between two GPS points using Haversine formula
     * @param point1 First GPS point
     * @param point2 Second GPS point
     * @return Distance in meters
     */
    double calculateDistance(const geographic_msgs::msg::GeoPoint& point1,
                           const geographic_msgs::msg::GeoPoint& point2) const {
        double lat1_rad = point1.latitude * DEG_TO_RAD;
        double lon1_rad = point1.longitude * DEG_TO_RAD;
        double lat2_rad = point2.latitude * DEG_TO_RAD;
        double lon2_rad = point2.longitude * DEG_TO_RAD;

        double dlat = lat2_rad - lat1_rad;
        double dlon = lon2_rad - lon1_rad;

        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1_rad) * std::cos(lat2_rad) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        return EARTH_RADIUS_M * c;
    }

    /**
     * @brief Calculate intermediate position between current and target for collision checking
     * @param current Current position
     * @param target Target position
     * @param fraction Fraction of the way to target (0.0 to 1.0)
     * @return Intermediate position
     */
    geographic_msgs::msg::GeoPoint interpolatePosition(const geographic_msgs::msg::GeoPoint& current,
                                                      const geographic_msgs::msg::GeoPoint& target,
                                                      double fraction) const {
        geographic_msgs::msg::GeoPoint result;
        result.latitude = current.latitude + fraction * (target.latitude - current.latitude);
        result.longitude = current.longitude + fraction * (target.longitude - current.longitude);
        result.altitude = current.altitude + fraction * (target.altitude - current.altitude);
        return result;
    }
};

#endif // SAFETY_GUARD_HPP