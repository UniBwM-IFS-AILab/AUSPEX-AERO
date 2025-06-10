#ifndef VEHICLE_STATUS_LISTENER_HPP
#define VEHICLE_STATUS_LISTENER_HPP

#include <string>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
using std::placeholders::_1;

class VehicleStatusListener_Base : public rclcpp::Node{
public:

    VehicleStatusListener_Base(const std::string &node_name) : rclcpp::Node(node_name) {
    }

    virtual ~VehicleStatusListener_Base() = default;

    /**
     * @brief gets the recent gps position.
     */
    virtual px4_msgs::msg::BatteryStatus::SharedPtr get_battery_msg() = 0;

    /**
     * @brief gets the recent gps position.
     */
	virtual px4_msgs::msg::VehicleStatus::SharedPtr get_VehicleStatus_msg() = 0;

    /**
     * @brief gets the recent gps position.
     */
	virtual bool get_Paused_from_Extern_msg() = 0;

    /**
     * @brief gets the recent gps position.
     */
	virtual void set_Paused_from_Extern_msg(bool new_paused) = 0;
};

#endif