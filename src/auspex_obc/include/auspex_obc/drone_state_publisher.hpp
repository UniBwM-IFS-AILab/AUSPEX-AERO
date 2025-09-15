#ifndef DRONE_STATE_PUBLISHER_HPP
#define DRONE_STATE_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include "auspex_fci/status_listener_base.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nlohmann/json.hpp>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

using json = nlohmann::json;

class DroneStatePublisher : public rclcpp::Node {
public:
    DroneStatePublisher(std::string name_prefix, json& platform_config): Node(name_prefix+ "_" + "drone_state_publisher") {
        name_prefix_ = name_prefix;

        json sensor_spec;
        if (platform_config.contains("sensors") && !platform_config["sensors"].empty()) {
            if (platform_config["sensors"][0].contains("specifications")) {
            sensor_spec = platform_config["sensors"][0]["specifications"];
            }
        }
        try{

            sensor_position_.x = sensor_spec["sensor_position"]["x"].get<double>();
            sensor_position_.y = sensor_spec["sensor_position"]["y"].get<double>();
            sensor_position_.z = sensor_spec["sensor_position"]["z"].get<double>();

            sensor_orientation_.x = sensor_spec["sensor_orientation"]["roll"].get<double>();
            sensor_orientation_.y = sensor_spec["sensor_orientation"]["pitch"].get<double>();
            sensor_orientation_.z = sensor_spec["sensor_orientation"]["yaw"].get<double>();

            fov_hor_ = sensor_spec["fov"]["horizontal"]["max"].get<int>();
            fov_vert_ = sensor_spec["fov"]["vertical"]["max"].get<int>();

        }catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON Parse Error: %s", e.what());
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Runtime Error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }

        rmw_qos_profile_t sensor_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(
            rclcpp::QoSInitialization(sensor_profile.history, sensor_profile.depth),
            sensor_profile
        );
        pub_ = this->create_publisher<PlatformState>("/platform_state", sensor_qos);

        // Set up a timer to publish every 500ms (2 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DroneStatePublisher::publishDroneStateMessage, this));
        timer_->cancel();
    }

    void publishDroneStateMessage() {
        // Create a DroneStatus message
        auto msg = PlatformState();
        if(this->gps_listener_ == nullptr){
            return;
        }

        // Fill in header information
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        // Fill in the platform info
        msg.platform_id = name_prefix_;      // drone ID
        msg.team_id = "drone_team";          // team ID

        // Set platform GPS position (latitude, longitude, altitude in WGS84)
        msg.platform_gps_position.latitude = this->gps_listener_->get_recent_gps_msg()->latitude_deg;  // Latitude
        msg.platform_gps_position.longitude = this->gps_listener_->get_recent_gps_msg()->longitude_deg;  // Longitude
        msg.platform_gps_position.altitude = this->gps_listener_->get_recent_gps_msg()->absolute_altitude_m;  // Altitude in meters amsl

        // Set platform pose in world coordinates (position + orientation)
        msg.platform_pose.position.x = this->gps_listener_->get_recent_ned_msg()->position_body.x_m; // X position in meters
        msg.platform_pose.position.y = this->gps_listener_->get_recent_ned_msg()->position_body.y_m; // Y position in meters
        msg.platform_pose.position.z = this->gps_listener_->get_recent_ned_msg()->position_body.z_m; // Z position (altitude) in meters agl

        double qx = gps_listener_->get_recent_ned_msg()->q.x;
        double qy = gps_listener_->get_recent_ned_msg()->q.y;
        double qz = gps_listener_->get_recent_ned_msg()->q.z;
        double qw = gps_listener_->get_recent_ned_msg()->q.w;

        if (std::isnan(qx) || std::isnan(qy) || std::isnan(qz) || std::isnan(qw)) {
            RCLCPP_WARN(this->get_logger(), "Quaternion contains NaN, skipping publish.");
            return;
        }

        msg.platform_pose.orientation.x = qx;
        msg.platform_pose.orientation.y = qy;
        msg.platform_pose.orientation.z = qz;
        msg.platform_pose.orientation.w = qw;

        // Set platform status
        msg.platform_status = this->gps_listener_->get_recent_platform_state();

        // Set sensor info
        msg.sensor_id = name_prefix_ + "camera_0";
        msg.sensor_position = sensor_position_;

        // Set the Field of View (FOV) and zoom level
        msg.fov_hor = fov_hor_;
        msg.fov_vert = fov_vert_;
        msg.zoom_level = 0;

        // Set gimbal orientation (direction the camera is looking) //TODO get from actual Gimbal
        msg.gimbal_orientation = sensor_orientation_;

        tf2::Quaternion q(qx, qy, qz, qw);
        tf2::Matrix3x3 m(q);
        // Gimbal
        msg.elevation_angle = std::asin(-m[2][0]) * 180.0 / M_PI; // in degrees to ground
        msg.azimuth_angle = std::atan2(m[1][0], m[0][0]) * 180.0 / M_PI; // in degrees to North

        // Set sensor mode (0 for EO, 1 for IR)
        msg.sensor_mode.value = 0;       // Electro-optical sensor

        //Battery Information
        auto current_battery_msg = this->status_listener_->get_battery_msg();
        msg.battery_state.voltage = std::isnan(current_battery_msg->voltage_v) ? 0.0f : current_battery_msg->voltage_v;
        msg.battery_state.current = std::isnan(current_battery_msg->current_battery_a) ? 0.0f : current_battery_msg->current_battery_a;
        msg.battery_state.percentage = std::isnan(current_battery_msg->remaining_percent) ? 0.0f : current_battery_msg->remaining_percent;

        // Publish the message
        pub_->publish(msg);
    }

    void start_publish(std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener, std::shared_ptr<VehicleStatusListener_Base> status_listener){
		if (!timer_running_) {
			this->gps_listener_ = gps_listener;
            this->status_listener_ = status_listener;
			timer_->reset();  // Start the timer
			timer_running_ = true;
			RCLCPP_INFO(this->get_logger(), "Drone State Publisher started.");
		}
	}
    void stop_publish(){
        if (timer_running_) {
            timer_->cancel();  // Stop the timer
            timer_running_ = false;
            RCLCPP_INFO(this->get_logger(), "Drone State Publisher stopped.");
        }
    }

private:
    rclcpp::Publisher<PlatformState>::SharedPtr pub_;
	std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener_;
	std::shared_ptr<VehicleStatusListener_Base> status_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool timer_running_ = false;
    std::string name_prefix_;

    geometry_msgs::msg::Point sensor_position_;
    geometry_msgs::msg::Vector3 sensor_orientation_;
    int fov_hor_;
    int fov_vert_;
};

#endif