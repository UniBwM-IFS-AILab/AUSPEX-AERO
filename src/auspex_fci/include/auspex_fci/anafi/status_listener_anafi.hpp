#ifndef STATUS_LISTENER_ANAFI
#define STATUS_LISTENER_ANAFI

#include "auspex_fci/status_listener_base.hpp"

class VehicleStatusListener_ANAFI : public VehicleStatusListener_Base {
public:

    VehicleStatusListener_ANAFI(std::string name_prefix) : VehicleStatusListener_Base(name_prefix + "_vehicle_status_listener_anafi") {
        mavsdk::Telemetry::Battery empty_bat_msg{};
		recent_battery_msg = std::make_shared<mavsdk::Telemetry::Battery>(std::move(empty_bat_msg));

		arming_state_ = false;

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    }

     /**
     * @brief gets the recent gps position.
     */
    std::shared_ptr<mavsdk::Telemetry::Battery> get_battery_msg() override {
        return recent_battery_msg;
    }

    /**
     * @brief gets the recent gps position.
     */
	bool get_arming_state() override {
        return arming_state_;
    }

    /**
     * @brief gets whether the vehicle is paused from an external command.
     */
	bool get_paused_from_extern() override {
        return paused_from_extern;
    }

    /**
     * @brief sets whether the vehicle is paused from an external command.
     */
	void set_paused_from_extern(bool new_paused) override {
        paused_from_extern = new_paused;
    }

private:

    std::shared_ptr<mavsdk::Telemetry::Battery> recent_battery_msg;
    bool arming_state_ = false;
	bool paused_from_extern = false;
};


#endif