#ifndef STATUS_LISTENER_MAVSDK
#define STATUS_LISTENER_MAVSDK

#include "auspex_fci/status_listener_base.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugin_base.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mutex>

class VehicleStatusListener_MAVSDK : public VehicleStatusListener_Base {
public:
    VehicleStatusListener_MAVSDK(std::string name_prefix, std::shared_ptr<mavsdk::System> system, std::string FC_TYPE)
        : VehicleStatusListener_Base(name_prefix + "_vehicle_status_listener_mavsdk") {
        mavsdk::Telemetry::Battery empty_bat_msg{};
        recent_battery_msg = std::make_shared<mavsdk::Telemetry::Battery>(std::move(empty_bat_msg));

        arming_state_ = false;

        system_ = system;
        if (system_) {
            telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
            telemetry_->subscribe_battery([this](mavsdk::Telemetry::Battery battery) {
                std::lock_guard<std::mutex> lock(mutex_);
                *recent_battery_msg = battery;
            });

            telemetry_->subscribe_armed([this](bool is_armed) {
                std::lock_guard<std::mutex> lock(mutex_);
                arming_state_ = is_armed;
            });

            telemetry_->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode flight_mode) {
               // RCLCPP_INFO(this->get_logger(), "Flight mode changed: %d", static_cast<int>(flight_mode));
            });
        } else {
            RCLCPP_ERROR(this->get_logger(), "No MAVSDK system found, status listener cannot subscribe to telemetry.");
        }
    }

    std::shared_ptr<mavsdk::Telemetry::Battery> get_battery_msg() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return recent_battery_msg;
    }

    bool get_arming_state() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return arming_state_;
    }

    bool get_paused_from_extern() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return paused_from_extern;
    }

    void set_paused_from_extern(bool new_paused) override {
        std::lock_guard<std::mutex> lock(mutex_);
        paused_from_extern = new_paused;
    }

private:
    std::mutex mutex_;

    std::shared_ptr<mavsdk::Telemetry::Battery> recent_battery_msg;
    bool arming_state_ = false;

    std::shared_ptr<mavsdk::System>          system_{nullptr};
    std::shared_ptr<mavsdk::Telemetry>       telemetry_;

    bool paused_from_extern = false;
};

#endif
