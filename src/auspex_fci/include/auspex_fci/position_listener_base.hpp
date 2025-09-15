#ifndef VEHICLE_POSITION_LISTENER_HPP
#define VEHICLE_POSITION_LISTENER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mavsdk/plugins/telemetry/telemetry.h>

class VehicleGlobalPositionListener_Base : public rclcpp::Node{
public:

    VehicleGlobalPositionListener_Base(const std::string &node_name) : rclcpp::Node(node_name) {
    }

    virtual ~VehicleGlobalPositionListener_Base() = default;

    /**
     * @brief checks if one gps update is already made. To init FC one gps position is needed.
     */
    virtual bool get_first_gps_future() = 0;

    /**
     * @brief gets the recent gps position.
     */
	virtual std::shared_ptr<mavsdk::Telemetry::Position> get_recent_gps_msg() = 0;

     /**
	* @brief Gets the home position
	*/
    virtual void update_home_position() = 0;

    /**
     * @brief Gets the recent ned position.
     */
	virtual std::shared_ptr<mavsdk::Telemetry::Odometry> get_recent_ned_msg() = 0;

    /**
     * @brief Gets the recent home position.
     */
	virtual std::shared_ptr<mavsdk::Telemetry::Position> get_recent_home_msg() = 0;

     /**
     * @brief Gets the recent platform state.
     */
     virtual std::string get_recent_platform_state() = 0;

    /**
     * @brief Sets the recent platform state.
     */
     virtual void set_recent_platform_state(std::string new_platform_state) = 0;

};

#endif