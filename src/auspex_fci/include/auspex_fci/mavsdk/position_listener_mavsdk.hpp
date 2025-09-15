#ifndef POSITION_LISTENER_MAVSDK
#define POSITION_LISTENER_MAVSDK

#include "auspex_fci/position_listener_base.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugin_base.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param/param.h>
#include <Eigen/Dense>
#include <cmath>
#include <mutex>

class VehicleGlobalPositionListener_MAVSDK : public VehicleGlobalPositionListener_Base{
public:
    VehicleGlobalPositionListener_MAVSDK(std::string name_prefix, std::shared_ptr<mavsdk::System> system, std::string FC_TYPE)
        : VehicleGlobalPositionListener_Base(name_prefix + "_" + "vehicle_global_position_listener_mavsdk") {
        mavsdk::Telemetry::Position empty_gps_msg{};
        recent_gps_msg = std::make_shared<mavsdk::Telemetry::Position>(std::move(empty_gps_msg));

        mavsdk::Telemetry::Odometry empty_ned_msg{};
        recent_ned_msg = std::make_shared<mavsdk::Telemetry::Odometry>(std::move(empty_ned_msg));

        mavsdk::Telemetry::Position empty_home_msg{};
        recent_home_msg = std::make_shared<mavsdk::Telemetry::Position>(std::move(empty_home_msg));

        recent_gps_msg->latitude_deg  = -1;
        recent_gps_msg->longitude_deg  = -1;
        recent_gps_msg->absolute_altitude_m  = -1;
        recent_gps_msg->relative_altitude_m   = -1;

        system_ = system;
        if (system_) {
            telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
            telemetry_->set_rate_position(10.0);

            telemetry_->subscribe_position([this](mavsdk::Telemetry::Position position) {
                std::lock_guard<std::mutex> lock(_mutex);
                *recent_gps_msg = position;
            });

            if(FC_TYPE.find("ARDUPILOT") != std::string::npos) {
                // ArduPilot: Use position_velocity_ned + attitude_euler to construct odometry
                telemetry_->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed pos_vel_ned) {
                    std::lock_guard<std::mutex> lock(_mutex);
                    recent_pos_vel_ned = pos_vel_ned;
                    update_odometry_from_components();
                });

                telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle euler_angle) {
                    std::lock_guard<std::mutex> lock(_mutex);
                    recent_euler_angle = euler_angle;
                    update_odometry_from_components();
                });

            } else {
                // PX4: Use direct odometry subscription
                telemetry_->subscribe_odometry([this](mavsdk::Telemetry::Odometry odometry_msg) {
                    std::lock_guard<std::mutex> lock(_mutex);
                    *recent_ned_msg = odometry_msg;
                });
            }

        }else {
            RCLCPP_ERROR(this->get_logger(), "No MAVSDK system found, position listener cannot subscribe to telemetry.");
        }
    }

    /**
     * @brief Helper method to combine position_velocity_ned and attitude_euler into odometry (for ArduPilot)
     */
    void update_odometry_from_components() {
        // Only update if we have both position/velocity and attitude data
        if (recent_pos_vel_ned.position.north_m != 0.0 || recent_pos_vel_ned.position.east_m != 0.0 ||
            recent_euler_angle.roll_deg != 0.0 || recent_euler_angle.pitch_deg != 0.0 || recent_euler_angle.yaw_deg != 0.0) {

            // Fill position in NED frame
            recent_ned_msg->position_body.x_m = recent_pos_vel_ned.position.north_m;
            recent_ned_msg->position_body.y_m = recent_pos_vel_ned.position.east_m;
            recent_ned_msg->position_body.z_m = recent_pos_vel_ned.position.down_m;

            // Fill velocity in NED frame
            recent_ned_msg->velocity_body.x_m_s = recent_pos_vel_ned.velocity.north_m_s;
            recent_ned_msg->velocity_body.y_m_s = recent_pos_vel_ned.velocity.east_m_s;
            recent_ned_msg->velocity_body.z_m_s = recent_pos_vel_ned.velocity.down_m_s;

            // Convert Euler angles to quaternion using Eigen
            // Convert degrees to radians
            float roll_rad = recent_euler_angle.roll_deg * M_PI / 180.0f;
            float pitch_rad = recent_euler_angle.pitch_deg * M_PI / 180.0f;
            float yaw_rad = recent_euler_angle.yaw_deg * M_PI / 180.0f;

            // Create quaternion from Euler angles (ZYX rotation sequence)
            Eigen::Quaternionf q =
                Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX());

            // Fill quaternion attitude
            recent_ned_msg->q.w = q.w();
            recent_ned_msg->q.x = q.x();
            recent_ned_msg->q.y = q.y();
            recent_ned_msg->q.z = q.z();
        }
    }


    void update_home_position() override {
        std::lock_guard<std::mutex> lock(_mutex);

        telemetry_->get_gps_global_origin_async([this](mavsdk::Telemetry::Result result, mavsdk::Telemetry::GpsGlobalOrigin origin) {
                if(origin.latitude_deg != 0.0 && origin.longitude_deg != 0.0 && origin.altitude_m != 0.0 &&
                    origin.latitude_deg != -1.0 && origin.longitude_deg != -1.0 && origin.altitude_m != -1.0 &&
                    !std::isnan(origin.latitude_deg) && !std::isnan(origin.longitude_deg) && !std::isnan(origin.altitude_m)) {

                    recent_home_msg->latitude_deg = origin.latitude_deg;
                    recent_home_msg->longitude_deg = origin.longitude_deg;
                    recent_home_msg->absolute_altitude_m = origin.altitude_m;
                    gps_init_set.exchange(true);

                }
            }
        );
    }

    bool get_first_gps_future() override {
        return gps_init_set.load();
    }

    std::shared_ptr<mavsdk::Telemetry::Position> get_recent_gps_msg() override {
        std::lock_guard<std::mutex> lock(_mutex);
        return recent_gps_msg;
    }

    std::shared_ptr<mavsdk::Telemetry::Odometry> get_recent_ned_msg() override {
        std::lock_guard<std::mutex> lock(_mutex);
        return recent_ned_msg;
    }

    std::shared_ptr<mavsdk::Telemetry::Position> get_recent_home_msg() override {
        std::lock_guard<std::mutex> lock(_mutex);
        return recent_home_msg;
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
    std::shared_ptr<mavsdk::Telemetry::Position> recent_gps_msg;
    std::shared_ptr<mavsdk::Telemetry::Odometry> recent_ned_msg;
    std::shared_ptr<mavsdk::Telemetry::Position> recent_home_msg;

    // ArduPilot-specific data for constructing odometry
    mavsdk::Telemetry::PositionVelocityNed recent_pos_vel_ned{};
    mavsdk::Telemetry::EulerAngle recent_euler_angle{};

    std::atomic<bool> gps_init_set{false};

    std::shared_ptr<mavsdk::System> system_{nullptr};
    std::shared_ptr<mavsdk::Telemetry> telemetry_;

    std::string platform_state_ = "INACTIVE";
};

#endif
