#ifndef STATUS_LISTENER_PX4
#define STATUS_LISTENER_PX4

#include "auspex_fci/status_listener_base.hpp"
#include <mutex>

class VehicleStatusListener_PX4 : public VehicleStatusListener_Base {
public:
    VehicleStatusListener_PX4(std::string name_prefix = "")
        : VehicleStatusListener_Base(name_prefix + "_vehicle_status_listener_px4") {
        px4_msgs::msg::BatteryStatus empty_bat_msg{};
        recent_battery_msg = std::make_shared<px4_msgs::msg::BatteryStatus>(std::move(empty_bat_msg));

        px4_msgs::msg::VehicleStatus empty_stat_msg{};
        recent_status_msg = std::make_shared<px4_msgs::msg::VehicleStatus>(std::move(empty_stat_msg));

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        vehicle_command_listener_ = this->create_subscription<px4_msgs::msg::VehicleCommand>(
            name_prefix + "/fmu/out/vehicle_command", qos,
            std::bind(&VehicleStatusListener_PX4::vehicle_command_callback, this, _1));

        battery_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
            name_prefix + "/fmu/out/battery_status", qos,
            [this](const px4_msgs::msg::BatteryStatus::UniquePtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                recent_battery_msg = std::make_shared<px4_msgs::msg::BatteryStatus>(*msg);
            });

        vehicle_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            name_prefix + "/fmu/out/vehicle_status", qos,
            [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                recent_status_msg = std::make_shared<px4_msgs::msg::VehicleStatus>(*msg);
            });
    }

    void vehicle_command_callback(const px4_msgs::msg::VehicleCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        int source = msg->source_system;
        if (source == 255 && msg->command == 192 && !paused_from_extern) {
            RCLCPP_INFO(this->get_logger(), "QGroundcontrol send a command -> pausing execution on OBC.");
            paused_from_extern = true;
        } else if (source == 255 && paused_from_extern && msg->command == 176) {
            paused_from_extern = false;
            RCLCPP_INFO(this->get_logger(), "Resuming Mission");
        }
    }

    px4_msgs::msg::BatteryStatus::SharedPtr get_battery_msg() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return recent_battery_msg;
    }

    px4_msgs::msg::VehicleStatus::SharedPtr get_VehicleStatus_msg() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return recent_status_msg;
    }

    bool get_Paused_from_Extern_msg() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return paused_from_extern;
    }

    void set_Paused_from_Extern_msg(bool new_paused) override {
        std::lock_guard<std::mutex> lock(mutex_);
        paused_from_extern = new_paused;
    }

private:
    std::mutex mutex_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_listener_;
    px4_msgs::msg::BatteryStatus::SharedPtr recent_battery_msg;
    px4_msgs::msg::VehicleStatus::SharedPtr recent_status_msg;
    bool paused_from_extern = false;
};

#endif
