#ifndef VEHICLE_POSITION_LISTENER_HPP
#define VEHICLE_POSITION_LISTENER_HPP

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/home_position.hpp>

class VehicleGlobalPositionListener_Base : public rclcpp::Node{
public:

    VehicleGlobalPositionListener_Base(const std::string &node_name) : rclcpp::Node(node_name) {
    }

    virtual ~VehicleGlobalPositionListener_Base() = default; 

    /**
     * @brief checks if one gps update is already made. To init FC one gps position is needed.
     */
    virtual std::future<bool> get_next_gps_future() = 0;

    /**
     * @brief gets the recent gps position.
     */
	virtual px4_msgs::msg::VehicleGlobalPosition::SharedPtr get_recent_gps_msg() = 0;

    /**
     * @brief Gets the recent ned position.
     */
	virtual px4_msgs::msg::VehicleOdometry::SharedPtr get_recent_ned_msg() = 0;

    /**
     * @brief Gets the recent home position.
     */
	virtual px4_msgs::msg::HomePosition::SharedPtr get_recent_home_msg() = 0;

};

#endif