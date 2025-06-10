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
                std::lock_guard<std::mutex> lock(_mutex);
                recent_gps_msg = std::make_shared<px4_msgs::msg::VehicleGlobalPosition>(*msg);
                if (!gps_promise_set_gps)
                {
                    gps_promise_gps.set_value(true);
                    gps_promise_set_gps = true;
                }
            });

        subscription_ned = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            name_prefix + "/fmu/out/vehicle_odometry", qos,
            [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
                std::lock_guard<std::mutex> lock(_mutex);
                recent_ned_msg = std::make_shared<px4_msgs::msg::VehicleOdometry>(*msg);
            });

        subscription_home = this->create_subscription<px4_msgs::msg::HomePosition>(
            name_prefix + "/fmu/out/home_position", qos,
            [this](const px4_msgs::msg::HomePosition::UniquePtr msg) {
                std::lock_guard<std::mutex> lock(_mutex);
                recent_home_msg = std::make_shared<px4_msgs::msg::HomePosition>(*msg);
            });
    }

    std::future<bool> get_next_gps_future() override {
        std::lock_guard<std::mutex> lock(_mutex);

        if(gps_promise_set_gps){
            gps_promise_set_gps = false;
            gps_promise_gps = std::promise<bool>();
        }
        return gps_promise_gps.get_future();
    }

    px4_msgs::msg::VehicleGlobalPosition::SharedPtr get_recent_gps_msg() override {
        std::lock_guard<std::mutex> lock(_mutex);
        return recent_gps_msg;
    }

    px4_msgs::msg::VehicleOdometry::SharedPtr get_recent_ned_msg() override {
        std::lock_guard<std::mutex> lock(_mutex);
        return recent_ned_msg;
    }

    px4_msgs::msg::HomePosition::SharedPtr get_recent_home_msg() override {
        std::lock_guard<std::mutex> lock(_mutex);
        return recent_home_msg;
    }

    void set_recent_home_msg() override {
        std::lock_guard<std::mutex> lock(_mutex);
        recent_home_msg->lat = recent_gps_msg->lat;
        recent_home_msg->lon = recent_gps_msg->lon;
        recent_home_msg->alt = recent_gps_msg->alt;
    }

    std::string get_recent_platform_state() override {
        return platform_state_;
    }

    void set_recent_platform_state(std::string new_platform_state) override{
        std::lock_guard<std::mutex> lock(_mutex);
        platform_state_ = new_platform_state;
    }

private:
    std::mutex _mutex;
    px4_msgs::msg::VehicleGlobalPosition::SharedPtr recent_gps_msg;
    px4_msgs::msg::VehicleOdometry::SharedPtr recent_ned_msg;
    px4_msgs::msg::HomePosition::SharedPtr recent_home_msg;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_gps;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_ned;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr subscription_home;

    std::promise<bool> gps_promise_gps;
    bool gps_promise_set_gps = false;


    std::string platform_state_ = "INACTIVE";
};

#endif
