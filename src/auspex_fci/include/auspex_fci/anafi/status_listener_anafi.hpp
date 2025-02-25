#ifndef STATUS_LISTENER_ANAFI
#define STATUS_LISTENER_ANAFI

#include "auspex_fci/status_listener_base.hpp"

class VehicleStatusListener_ANAFI : public VehicleStatusListener_Base {
public:

    VehicleStatusListener_ANAFI(std::string name_prefix = "") : VehicleStatusListener_Base(name_prefix + "_vehicle_status_listener_anafi") {
        px4_msgs::msg::BatteryStatus empty_bat_msg{};
		recent_battery_msg = std::make_shared<px4_msgs::msg::BatteryStatus>(std::move(empty_bat_msg));
		
		px4_msgs::msg::VehicleStatus empty_stat_msg{};
		recent_status_msg = std::make_shared<px4_msgs::msg::VehicleStatus>(std::move(empty_stat_msg));

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    }

     /**
     * @brief gets the recent gps position.
     */
    px4_msgs::msg::BatteryStatus::SharedPtr get_battery_msg() override {
        return recent_battery_msg;
    }

    /**
     * @brief gets the recent gps position.
     */
	px4_msgs::msg::VehicleStatus::SharedPtr get_VehicleStatus_msg() override {
        return recent_status_msg;
    }

    /**
     * @brief gets the recent gps position.
     */
	bool get_Paused_from_Extern_msg() override {
        return paused_from_extern;
    }

    /**
     * @brief gets the recent gps position.
     */
	void set_Paused_from_Extern_msg(bool new_paused) override {
        paused_from_extern = new_paused;
    }

private:

	px4_msgs::msg::BatteryStatus::SharedPtr recent_battery_msg;
	px4_msgs::msg::VehicleStatus::SharedPtr recent_status_msg;
	bool paused_from_extern = false;
};


#endif