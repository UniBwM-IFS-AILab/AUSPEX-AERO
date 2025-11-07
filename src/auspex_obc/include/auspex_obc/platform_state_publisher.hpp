#ifndef DRONE_STATE_PUBLISHER_HPP
#define DRONE_STATE_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include "auspex_fci/status_listener_base.hpp"
#include "auspex_msgs/msg/platform_state.hpp"
#include "auspex_msgs/msg/platform_capabilities.hpp"
#include "auspex_msgs/msg/sensor_mode.hpp"
#include "auspex_msgs/msg/platform_class.hpp"
#include "auspex_msgs/msg/sensor_capabilities.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nlohmann/json.hpp>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <string.h>

using json = nlohmann::json;
using PlatformState = auspex_msgs::msg::PlatformState;
using PlatformClass = auspex_msgs::msg::PlatformClass;
using PlatformCapabilities = auspex_msgs::msg::PlatformCapabilities;
using SensorMode = auspex_msgs::msg::SensorMode;
using SensorCapabilities = auspex_msgs::msg::SensorCapabilities;

class PlatformStatePublisher : public rclcpp::Node {
public:
    PlatformStatePublisher(std::string platform_id, std::string platform_ip, json& platform_config): Node(platform_id + "_" + "state_pub") {
        platform_id_ = platform_id;
        platform_ip_ = platform_ip;

        json sensor_spec;
        if (platform_config.contains("sensors") && !platform_config["sensors"].empty()) {
            if (platform_config["sensors"][0].contains("specifications")) {
                sensor_spec = platform_config["sensors"][0]["specifications"];
            }
        }

        try{
            /*Build for Capabilities*/
            std::vector<SensorCapabilities> sensor_capabilities_msg;
            for(auto &sensor : platform_config["sensors"]){
                auto sensor_msg = SensorCapabilities();
                sensor_msg.sensor_id = sensor["id"];

                auto sensor_mode = SensorMode();
                if(sensor["type"] == "eo_camera"){
                    sensor_mode.value = SensorMode::SENSOR_MODE_EO;
                }else if(sensor["type"] == "ir_camera") {
                    sensor_mode.value = SensorMode::SENSOR_MODE_IR;
                }
                sensor_msg.sensor_mode = sensor_mode;

                sensor_msg.fov_hor_min = sensor["specifications"]["fov"]["horizontal"]["min"];
                sensor_msg.fov_hor_max = sensor["specifications"]["fov"]["horizontal"]["max"];
                sensor_msg.fov_vert_min = sensor["specifications"]["fov"]["vertical"]["min"];
                sensor_msg.fov_vert_max = sensor["specifications"]["fov"]["vertical"]["max"];
                sensor_msg.image_width = sensor["specifications"]["image_size"]["width"];
                sensor_msg.image_height = sensor["specifications"]["image_size"]["height"];

                sensor_capabilities_msg.push_back(sensor_msg);
            }

            platform_capabilities_msg_.platform_id = platform_id_;
            platform_capabilities_msg_.model_info = platform_config["platform_details"]["model"];

            platform_capabilities_msg_.max_flight_duration = platform_config["platform_details"]["max_flight_duration"];
            platform_capabilities_msg_.max_flight_height = platform_config["platform_details"]["max_flight_height"];
            platform_capabilities_msg_.max_velocity = platform_config["platform_details"]["max_velocity"];
            platform_capabilities_msg_.turning_radius = platform_config["platform_details"]["turning_radius"];

            auto platform_class = PlatformClass();
            platform_class.value = PlatformClass::PLATFORM_CLASS_DRONE;
            platform_capabilities_msg_.platform_class = platform_class;
            platform_capabilities_msg_.sensor_caps = sensor_capabilities_msg;
            
            /*Build for Platform State*/ 
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
        platform_state_publisher_ = this->create_publisher<PlatformState>("/platform_state", sensor_qos);
        platform_capabilities_publisher_ = this->create_publisher<PlatformCapabilities>("platform_capabilities", 10);

        // Set up a timer to publish every 500ms (2 Hz)
        platform_state_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PlatformStatePublisher::publishPlatformStateMessage, this));
        platform_state_publisher_timer_->cancel();

        platform_capabilities_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&PlatformStatePublisher::publishCapabilitiesMessage, this));
        platform_capabilities_publisher_timer_->cancel();
        platform_capabilities_publisher_->publish(platform_capabilities_msg_);
    }

    void publishCapabilitiesMessage() {
        platform_capabilities_publisher_->publish(platform_capabilities_msg_);
    }

    void publishPlatformStateMessage() {
        // Create a DroneStatus message
        auto msg = PlatformState();
        if(this->gps_listener_ == nullptr){
            return;
        }

        // Fill in header information
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";

        // Fill in the platform info
        msg.platform_id = platform_id_;      // drone ID
        msg.platform_ip = platform_ip_;      // platform IP
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
        msg.sensor_id = platform_id_ + "camera_0";
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
        platform_state_publisher_->publish(msg);
    }

    void start_publish(std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener, std::shared_ptr<VehicleStatusListener_Base> status_listener){
		if (!timers_active_) {
			this->gps_listener_ = gps_listener;
            this->status_listener_ = status_listener;
			platform_capabilities_publisher_timer_->reset();  // Start the timer
			platform_state_publisher_timer_->reset();  // Start the timer
			timers_active_ = true;
			RCLCPP_INFO(this->get_logger(), "Platform State Publisher started.");
		}
	}
    void stop_publish(){
        if (timers_active_) {
            platform_capabilities_publisher_timer_->cancel();  // Stop the timer
            platform_state_publisher_timer_->cancel();  // Stop the timer
            timers_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Platform State Publisher stopped.");
        }
    }

private:
    rclcpp::Publisher<PlatformState>::SharedPtr platform_state_publisher_;
    rclcpp::Publisher<PlatformCapabilities>::SharedPtr platform_capabilities_publisher_;

	std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener_;
	std::shared_ptr<VehicleStatusListener_Base> status_listener_;

    rclcpp::TimerBase::SharedPtr platform_state_publisher_timer_;
    rclcpp::TimerBase::SharedPtr platform_capabilities_publisher_timer_;

    bool timers_active_ = false;

    std::string platform_id_;
    std::string platform_ip_;

    PlatformCapabilities platform_capabilities_msg_;

    geometry_msgs::msg::Point sensor_position_;
    geometry_msgs::msg::Vector3 sensor_orientation_;
    int fov_hor_;
    int fov_vert_;
};

#endif