#ifndef POSITION_LISTENER_PX4
#define POSITION_LISTENER_PX4

#include "auspex_fci/position_listener_base.hpp"
#include <mutex>

class VehicleGlobalPositionListener_PX4 : public VehicleGlobalPositionListener_Base{
public:
    VehicleGlobalPositionListener_PX4(std::string name_prefix = "") 
        : VehicleGlobalPositionListener_Base(name_prefix + "_" + "vehicle_global_position_listener_px4") {
        px4_msgs::msg::VehicleGlobalPosition empty_gps_msg{};
        recent_gps_msg = std::make_shared<px4_msgs::msg::VehicleGlobalPosition>(std::move(empty_gps_msg));
        
        recent_gps_msg->lat = -1;
        recent_gps_msg->lon = -1;
        recent_gps_msg->alt = -1;
        
        px4_msgs::msg::VehicleOdometry empty_ned_msg{};
        recent_ned_msg = std::make_shared<px4_msgs::msg::VehicleOdometry>(std::move(empty_ned_msg));
        
        px4_msgs::msg::HomePosition empty_home_msg{};
        recent_home_msg = std::make_shared<px4_msgs::msg::HomePosition>(std::move(empty_home_msg));

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        subscription_gps = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            name_prefix + "/fmu/out/vehicle_global_position", qos,
            [this](const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                recent_gps_msg = std::make_shared<px4_msgs::msg::VehicleGlobalPosition>(*msg);
                if (!gps_promise_set)
                {
                    gps_promise.set_value(true);
                    gps_promise_set = true;
                }
            });

        subscription_ned = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            name_prefix + "/fmu/out/vehicle_odometry", qos,
            [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                recent_ned_msg = std::make_shared<px4_msgs::msg::VehicleOdometry>(*msg);
            });

        subscription_home = this->create_subscription<px4_msgs::msg::HomePosition>(
            name_prefix + "/fmu/out/home_position", qos,
            [this](const px4_msgs::msg::HomePosition::UniquePtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                recent_home_msg = std::make_shared<px4_msgs::msg::HomePosition>(*msg);
            });
    }

    std::future<bool> get_next_gps_future() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if(gps_promise_set){            
            gps_promise_set = false;
            gps_promise = std::promise<bool>();            
        }
        return gps_promise.get_future();
    }

    px4_msgs::msg::VehicleGlobalPosition::SharedPtr get_recent_gps_msg() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return recent_gps_msg;
    }

    px4_msgs::msg::VehicleOdometry::SharedPtr get_recent_ned_msg() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return recent_ned_msg;
    }

    px4_msgs::msg::HomePosition::SharedPtr get_recent_home_msg() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return recent_home_msg;
    }

private:
    std::mutex mutex_;
    px4_msgs::msg::VehicleGlobalPosition::SharedPtr recent_gps_msg;
    px4_msgs::msg::VehicleOdometry::SharedPtr recent_ned_msg;
    px4_msgs::msg::HomePosition::SharedPtr recent_home_msg;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_gps;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_ned;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr subscription_home;
    
    std::promise<bool> gps_promise;
    bool gps_promise_set = false;
};

#endif
