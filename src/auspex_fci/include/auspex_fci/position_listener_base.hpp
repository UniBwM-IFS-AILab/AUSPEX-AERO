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
    virtual void is_home_position_set() = 0;

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

    /**
     * @brief Set the home ground altitude AMSL
     * @param height_amsl The height above mean sea level in meters
     */
    void set_home_ground_altitude_amsl(double height_amsl) {
        home_ground_altitude_amsl = height_amsl;
    }

    /**
     * @brief Get the home ground altitude AMSL
     * @return The height above mean sea level in meters
     */
    double get_home_ground_altitude_amsl() const {
        return home_ground_altitude_amsl;
    }

    virtual double get_fc_height() = 0;

protected:
    double home_ground_altitude_amsl = 0.0; // Ground height in meters (for altitude conversion)

};

#endif