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
        //TODO: Get configs from platform_config

        rmw_qos_profile_t sensor_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(
            rclcpp::QoSInitialization(sensor_profile.history, sensor_profile.depth),
            sensor_profile
        );
        pub_ = this->create_publisher<PlatformState>("/drone_state", sensor_qos);

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
        msg.platform_gps_position.latitude = this->gps_listener_->get_recent_gps_msg()->lat;  // Latitude
        msg.platform_gps_position.longitude = this->gps_listener_->get_recent_gps_msg()->lon;  // Longitude
        msg.platform_gps_position.altitude = this->gps_listener_->get_recent_gps_msg()->alt;  // Altitude in meters amsl

        // Set platform pose in world coordinates (position + orientation)
        msg.platform_pose.position.x = this->gps_listener_->get_recent_ned_msg()->position[0]; // X position in meters
        msg.platform_pose.position.y = this->gps_listener_->get_recent_ned_msg()->position[1]; // Y position in meters
        msg.platform_pose.position.z = this->gps_listener_->get_recent_ned_msg()->position[2]; // Z position (altitude) in meters agl

        msg.platform_pose.orientation.x = this->gps_listener_->get_recent_ned_msg()->q[1];
        msg.platform_pose.orientation.y = this->gps_listener_->get_recent_ned_msg()->q[2];
        msg.platform_pose.orientation.z = this->gps_listener_->get_recent_ned_msg()->q[3];
        msg.platform_pose.orientation.w = this->gps_listener_->get_recent_ned_msg()->q[0];

        tf2::Quaternion q(gps_listener_->get_recent_ned_msg()->q[1],gps_listener_->get_recent_ned_msg()->q[2],gps_listener_->get_recent_ned_msg()->q[3],gps_listener_->get_recent_ned_msg()->q[0]);
        double roll, pitch,yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // Set platform status
        msg.platform_status = this->gps_listener_->get_recent_platform_state(); //Select from ...

        // Set sensor info
        msg.sensor_id = "Camera001";  // Demo sensor ID
        msg.sensor_position.x = 0.5;  // 0.5m to the right of the platform
        msg.sensor_position.y = 0.0;  // Centered on the platform
        msg.sensor_position.z = 0.0;  // At the same altitude as the platform

        // Set the Field of View (FOV) and zoom level
        msg.fov_hor = 60;  // 60 degrees horizontal FOV
        msg.fov_vert = 60;    // 60 degrees vertical FOV
        msg.zoom_level = 0;       // Zoom level 0

        // Set gimbal orientation
        msg.gimbal_orientation.x = 0.0;  // Roll
        msg.gimbal_orientation.y = -45.0; // Pitch:
        msg.gimbal_orientation.z = 0.0; // Yaw:

        // Set gimbal angles (elevation and azimuth)
        msg.elevation_angle = 0.0; //asin(-R31); of R with R = R_z * R_y * R_x R rotation matrices   // to ground
        msg.azimuth_angle = 0.0; //atan2(R21, R11);    // to true north

        // Set sensor mode (0 for EO, 1 for IR)
        msg.sensor_mode.value = 0;       // Electro-optical sensor

        //Battery Information
        auto current_battery_msg = this->status_listener_->get_battery_msg();
        msg.battery_state.voltage = current_battery_msg->voltage_v;
        msg.battery_state.current = current_battery_msg->current_a;
        msg.battery_state.charge = current_battery_msg->remaining_capacity_wh;
        msg.battery_state.capacity = current_battery_msg->capacity;
        msg.battery_state.design_capacity = current_battery_msg->full_charge_capacity_wh;
        msg.battery_state.percentage = current_battery_msg->remaining;
        for (int i = 0; i < current_battery_msg->voltage_cell_v.size(); ++i) {
            msg.battery_state.cell_voltage.push_back(current_battery_msg->voltage_cell_v[i]);
        }
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
};

#endif