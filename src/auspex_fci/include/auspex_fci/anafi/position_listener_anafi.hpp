#ifndef POSITION_LISTENER_ANAFI
#define POSITION_LISTENER_ANAFI

#include "auspex_fci/position_listener_base.hpp"

class VehicleGlobalPositionListener_ANAFI : public VehicleGlobalPositionListener_Base{
public:
    VehicleGlobalPositionListener_ANAFI(std::string name_prefix) : VehicleGlobalPositionListener_Base(name_prefix + "_" + "vehicle_global_position_listener_anafi") {
        mavsdk::Telemetry::Position empty_gps_msg{};
		recent_gps_msg = std::make_shared<mavsdk::Telemetry::Position>(std::move(empty_gps_msg));
		// initialize with negative values to check if initial message was received
		recent_gps_msg->latitude_deg = -1;
		recent_gps_msg->longitude_deg = -1;
		recent_gps_msg->absolute_altitude_m = -1;

		mavsdk::Telemetry::Odometry empty_ned_msg{};
		recent_ned_msg = std::make_shared<mavsdk::Telemetry::Odometry>(std::move(empty_ned_msg));

		mavsdk::Telemetry::Position empty_home_msg{};
		recent_home_msg = std::make_shared<mavsdk::Telemetry::Position>(std::move(empty_home_msg));

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    }

    /**
     * @brief checks if one gps update is already made. To init FC one gps position is needed.
     */
    bool get_first_gps_future() override {
        return gps_init_set.load();
    }

    void is_home_position_set() override {

    }

    /**
     * @brief gets the recent gps position.
     */
	std::shared_ptr<mavsdk::Telemetry::Position> get_recent_gps_msg() override {
        return recent_gps_msg;
    }

    /**
     * @brief Gets the recent ned position.
     */
	std::shared_ptr<mavsdk::Telemetry::Odometry> get_recent_ned_msg() override {
        return recent_ned_msg;
    }

    /**
     * @brief Gets the recent home position.
     */
	std::shared_ptr<mavsdk::Telemetry::Position> get_recent_home_msg() override {
        return recent_home_msg;
    }

    /**
     * @brief Gets the recent platform state.
     */
    std::string get_recent_platform_state() override {
        return "INACTIVE";
    }

    /**
     * @brief Sets the recent platform state.
     */
    void set_recent_platform_state(std::string new_platform_state) override {

    };

    double get_fc_height() override {
        return -1.0;
    }

private:
	std::shared_ptr<mavsdk::Telemetry::Position> recent_gps_msg;
	std::shared_ptr<mavsdk::Telemetry::Odometry> recent_ned_msg;
	std::shared_ptr<mavsdk::Telemetry::Position> recent_home_msg;

    std::atomic<bool> gps_init_set{false};
};


#endif