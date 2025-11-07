#ifndef EXTERNAL_DATA_LISTENER_HPP
#define EXTERNAL_DATA_LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "auspex_msgs/msg/platform_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <string>


using PlatformState = auspex_msgs::msg::PlatformState;

/**
 * @brief Data structure to store essential platform information
 */
struct PlatformData {
    std::string platform_id;
    std::string team_id;
    std::string platform_status;
    geometry_msgs::msg::Pose platform_pose;  // Position and orientation in world coordinates
    geographic_msgs::msg::GeoPoint platform_gps_position;  // GPS position (lat, lon, alt)
    std::string sensor_id;
    geometry_msgs::msg::Point sensor_position;
    geometry_msgs::msg::Vector3 gimbal_orientation;
    rclcpp::Time last_update;
    
    // Constructor
    PlatformData() : last_update(rclcpp::Time(0)) {}
};

/**
 * @brief Listens to platform_state topic and maintains information about other platforms
 * 
 * This class subscribes to the /platform_state topic and stores the most recent state
 * information for each platform (excluding the own platform). The stored data can be
 * used for collision avoidance and coordination between platforms.
 */
class ExternalDataListener : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param own_platform_id The platform ID of this node (to filter out own messages)
     */
    ExternalDataListener(const std::string& own_platform_id);

    /**
     * @brief Destructor
     */
    ~ExternalDataListener() = default;

    /**
     * @brief Get GPS positions of all known external platforms
     * @return Vector of GeoPoint positions (lat, lon, alt) for all external platforms
     */
    std::vector<geographic_msgs::msg::GeoPoint> getAllPlatformPositions() const;

    /**
     * @brief Get world coordinate poses of all known external platforms  
     * @return Vector of poses for all external platforms
     */
    std::vector<geometry_msgs::msg::Pose> getAllPlatformPoses() const;

    /**
     * @brief Get GPS positions of all known external platforms as Point messages
     * @return Vector of Point messages (longitude, latitude, altitude) for backward compatibility
     */
    std::vector<geometry_msgs::msg::Point> getAllPlatformGpsPositions() const;

    /**
     * @brief Get all known external platform IDs
     * @return Vector of platform ID strings
     */
    std::vector<std::string> getAllPlatformIds() const;

    /**
     * @brief Get platform data for a specific platform
     * @param platform_id The ID of the platform to get data for
     * @return Pointer to platform data, nullptr if platform not found
     */
    std::shared_ptr<const PlatformData> getPlatformData(const std::string& platform_id) const;

    /**
     * @brief Get the number of known external platforms
     * @return Number of platforms currently tracked
     */
    size_t getPlatformCount() const;

    /**
     * @brief Check if a specific platform is known
     * @param platform_id The ID to check for
     * @return True if platform is in the database
     */
    bool hasPlatform(const std::string& platform_id) const;

    /**
     * @brief Get platforms within a certain distance from a reference point
     * @param reference_pose Reference position to measure distance from
     * @param max_distance Maximum distance in meters
     * @return Vector of platform IDs within the specified distance
     */
    std::vector<std::string> getPlatformsWithinDistance(const geographic_msgs::msg::GeoPoint& reference_pose, double max_distance) const;

    /**
     * @brief Remove platforms that haven't been updated for a specified time
     * @param timeout_seconds Time threshold in seconds
     * @return Number of platforms removed
     */
    size_t removeOutdatedPlatforms(double timeout_seconds = 30.0);

private:
    /**
     * @brief Callback for platform state messages
     * @param msg Received platform state message
     */
    void platformStateCallback(const PlatformState::SharedPtr msg);

    /**
     * @brief Calculate Euclidean distance between two poses
     * @param pose1 First pose
     * @param pose2 Second pose
     * @return Distance in meters
     */
    double calculateDistance(const geographic_msgs::msg::GeoPoint& point1, const geographic_msgs::msg::GeoPoint& point2) const;

    // Member variables
    std::string own_platform_id_;
    rclcpp::Subscription<PlatformState>::SharedPtr subscription_;
    
    // Thread-safe storage for platform data
    mutable std::mutex platforms_mutex_;
    std::unordered_map<std::string, std::shared_ptr<PlatformData>> platforms_;
    
    // Timer for periodic cleanup of outdated platforms
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
};

#endif // EXTERNAL_DATA_LISTENER_HPP
