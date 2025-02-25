#ifndef POSITION_LISTENER_ANAFI
#define POSITION_LISTENER_ANAFI

#include "auspex_fci/position_listener_base.hpp"

class VehicleGlobalPositionListener_ANAFI : public VehicleGlobalPositionListener_Base{
public:
    VehicleGlobalPositionListener_ANAFI(std::string name_prefix = "") : VehicleGlobalPositionListener_Base(name_prefix + "_" + "vehicle_global_position_listener_anafi") {
        px4_msgs::msg::VehicleGlobalPosition empty_gps_msg{};
		recent_gps_msg = std::make_shared<px4_msgs::msg::VehicleGlobalPosition>(std::move(empty_gps_msg));
		// initialize with negative values to check if initial message was received
		recent_gps_msg->lat = -1;
		recent_gps_msg->lon = -1;
		recent_gps_msg->alt = -1;
		
		px4_msgs::msg::VehicleOdometry empty_ned_msg{};
		recent_ned_msg = std::make_shared<px4_msgs::msg::VehicleOdometry>(std::move(empty_ned_msg));
		
		px4_msgs::msg::HomePosition empty_home_msg{};
		recent_home_msg = std::make_shared<px4_msgs::msg::HomePosition>(std::move(empty_home_msg));

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
    }

    /**
     * @brief checks if one gps update is already made. To init FC one gps position is needed.
     */
    std::future<bool> get_next_gps_future() override {
        if(gps_promise_set){			
			gps_promise_set = false;
			gps_promise = std::promise<bool>();			
		}
		return gps_promise.get_future();
    }

    /**
     * @brief gets the recent gps position.
     */
	px4_msgs::msg::VehicleGlobalPosition::SharedPtr get_recent_gps_msg() override {
        return recent_gps_msg;
    }

    /**
     * @brief Gets the recent ned position.
     */
	px4_msgs::msg::VehicleOdometry::SharedPtr get_recent_ned_msg() override {
        return recent_ned_msg;
    }

    /**
     * @brief Gets the recent home position.
     */
	px4_msgs::msg::HomePosition::SharedPtr get_recent_home_msg() override {
        return recent_home_msg;
    }
private:
	px4_msgs::msg::VehicleGlobalPosition::SharedPtr recent_gps_msg;
	px4_msgs::msg::VehicleOdometry::SharedPtr recent_ned_msg;
	px4_msgs::msg::HomePosition::SharedPtr recent_home_msg;
	
	std::promise<bool> gps_promise;
	bool gps_promise_set = false;
};


#endif