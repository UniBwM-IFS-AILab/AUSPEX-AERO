#ifndef VEHICLE_STATUS_LISTENER_HPP
#define VEHICLE_STATUS_LISTENER_HPP

#include <string>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using std::placeholders::_1;

class VehicleStatusListener_Base : public rclcpp::Node{
public:

    VehicleStatusListener_Base(const std::string &node_name) : rclcpp::Node(node_name) {
    }

    virtual ~VehicleStatusListener_Base() = default;

    /**
     * @brief gets the recent gps position.
     */
    virtual std::shared_ptr<mavsdk::Telemetry::Battery> get_battery_msg() = 0;

    /**
     * @brief gets the recent gps position.
     */
	virtual bool get_arming_state() = 0;

    /**
     * @brief gets whether the vehicle is paused from an external command.
     */
	virtual bool get_paused_from_extern() = 0;

    /**
     * @brief sets whether the vehicle is paused from an external command.
     */
	virtual void set_paused_from_extern(bool new_paused) = 0;
};

#endif