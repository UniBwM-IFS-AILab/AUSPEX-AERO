#ifndef FC_INTERFACE_PX4
#define FC_INTERFACE_PX4

#include "auspex_fci/fc_interface_base.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "auspex_fci/status_listener_base.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class FC_Interface_PX4 : public FC_Interface_Base {
public:

    FC_Interface_PX4(std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener, std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener, std::string name_prefix = ""): FC_Interface_Base(name_prefix + "_" + "fc_interface_px4"){
        name_prefix_ = name_prefix;

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(name_prefix + "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(name_prefix + "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(name_prefix + "/fmu/in/vehicle_command", 10);

        vehicle_status_listener_ = vehicle_status_listener;
        position_listener_ = position_listener;
    }

    /**
     * @brief To get the gps converter. Must be here because one gps message needs to be send to initaliaze.
     */
    void set_gps_converter(std::shared_ptr<GeodeticConverter> gps_converter) override {
        gps_converter_ = gps_converter;
    }

    /**
	* @brief Send a command to set the home position of the vehicle (px4)
	*/
    void set_home_fc(double latitude, double longitude, double altitude) override {
        // |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0, 0, 0, 0, latitude, longitude, altitude);
    }

    /**
    * @brief Send a command to the vehicle to take off 10 meters <-- altitude is not relative to the ground
    */
    bool takeoff(double takeoff_height) override {
        // Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
        if(publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 0, position_listener_->get_recent_gps_msg()->lat, position_listener_->get_recent_gps_msg()->lon, takeoff_height)){
            RCLCPP_INFO(this->get_logger(), "Takeoff command send");
            return true;
        }
        return false;
    }

    /**
    * @brief Send a command to the vehicle to land
    */
    bool land(double latitude, double longitude, double altitude) override {
        if(publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0, 0, 0, latitude, longitude, altitude)){
            RCLCPP_INFO(this->get_logger(), "Landing command sent");
            return false;
        }
        return true;
    }

    /**
    * @brief Send a command to Arm the vehicle
    */
    void arm() override {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	    RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    /**
    * @brief Send a command to Disarm the vehicle
    */
    void disarm() override {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command send");
    }

    /**
    * @brief Send a command to enable position control (PX4)
    */
    bool publish_position_control_mode() override {
        return publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0,0,0,0,0,0);
    }

    /**
    * @brief Set FC to offboard control mode.
    */
    bool publish_offboard_control_mode() override {
        return publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0,0,0,0,0,0);
    }

    /**
    * @brief Send a heartbeat (PX4) and position control. (PX4)
    */
    void publish_offboard_heartbeat() override {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_control_mode_publisher_->publish(msg);
    }

    /**
    * @brief Publish a command to let the vehicle hover in a given ned position.
    */
    void hover_in_position(double hover_north, double hover_east, double hover_down, double roll, double pitch, double yaw) override {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        msg.position = {hover_north, hover_east, hover_down};
        msg.yaw = yaw; // [-PI:PI]

        publish_offboard_heartbeat(); //hold flight mode
        trajectory_setpoint_publisher_->publish(msg);
    }

    /**
    * @brief Publish a command to let the vehicle hover in the current position.
    */
    void hover_in_current_position() override {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q[1],position_listener_->get_recent_ned_msg()->q[2],position_listener_->get_recent_ned_msg()->q[3],position_listener_->get_recent_ned_msg()->q[0]);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        msg.position = {position_listener_->get_recent_ned_msg()->position[0], position_listener_->get_recent_ned_msg()->position[1], position_listener_->get_recent_ned_msg()->position[2]};
        msg.yaw = yaw; // [-PI:PI]
        this->publish_offboard_control_mode();
        this->publish_offboard_heartbeat(); //hold flight mode
        trajectory_setpoint_publisher_->publish(msg);
    }

    /**
    * @brief Publish a trajectory setpoint to a gps coordinate returns the distance left to the target while moving towards the target, dont decrease the distances, the target NED trajectory point should stay constant
    */
    double move_to_gps(double latitude, double longitude, double altitude, HEADING heading) override {
        // NED Coordinates for target
        double north, east, down;
        gps_converter_->geodetic2Ned(latitude, longitude, altitude, &north, &east, &down); // alt in above mean sea level

        double current_north, current_east, current_down;

        // receive current NED coordinates from PX4
        current_north = position_listener_->get_recent_ned_msg()->position[0];
        current_east = position_listener_->get_recent_ned_msg()->position[1];
        current_down = position_listener_->get_recent_ned_msg()->position[2];

        double delta_n = north - current_north, delta_e = east - current_east, delta_d = down - current_down;
        // discretize delta values
        delta_n = gps_converter_->discretize(delta_n,5.0);
        delta_e = gps_converter_->discretize(delta_e,5.0);
        delta_d = gps_converter_->discretize(delta_d,5.0);

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        north = gps_converter_->discretize(north,5.0);
        east = gps_converter_->discretize(east,5.0);
        down = gps_converter_->discretize(down,5.0);


        double yaw = 0.0;

        //Switch if drone should face next waypoint with front or not. else case is used for ascend and descend
        if(heading == HEADING::WAYPOINT){
            yaw = atan2(east -current_east,north - current_north); // [-PI:PI]
            yaw = gps_converter_->discretize(yaw,5.0);
        }else if(heading == HEADING::UNCHANGED){
            tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q[1],position_listener_->get_recent_ned_msg()->q[2],position_listener_->get_recent_ned_msg()->q[3],position_listener_->get_recent_ned_msg()->q[0]);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, yaw);
        }

        msg.position = {north, east, down};
        msg.yaw = yaw;

        publish_offboard_heartbeat();

        if(!vehicle_status_listener_->get_Paused_from_Extern_msg()){
            trajectory_setpoint_publisher_->publish(msg);
        }

        return gps_converter_->getDistance(delta_n, delta_e, delta_d);
    }

    /**
    * @brief Publish a trajectory setpoint to a ned coordinate
    */
    double move_to_ned(double north, double east, double down, double roll, double pitch, double yaw) override {
        // Current NED coordinates relative to origin
        double current_north, current_east, current_down;

        // receive current NED coordinates from PX4
        current_north = position_listener_->get_recent_ned_msg()->position[0];
        current_east = position_listener_->get_recent_ned_msg()->position[1];
        current_down = position_listener_->get_recent_ned_msg()->position[2];

        double delta_n = north - current_north, delta_e = east - current_east, delta_d = down - current_down;

        // discretize delta values
        delta_n = gps_converter_->discretize(delta_n,5.0);
        delta_e = gps_converter_->discretize(delta_e,5.0);
        delta_d = gps_converter_->discretize(delta_d,5.0);

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        north = gps_converter_->discretize(north,5.0);
        east = gps_converter_->discretize(east,5.0);
        down = gps_converter_->discretize(down,5.0);

        msg.position = {north, east, down};
        msg.yaw = yaw;

        publish_offboard_heartbeat();

        if(!vehicle_status_listener_->get_Paused_from_Extern_msg()){
            trajectory_setpoint_publisher_->publish(msg);
        }

        return gps_converter_->getDistance(delta_n, delta_e, delta_d);
    }

    /**
	* @brief Publish vehicle commands
	* @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
	* @param param1    Command parameter 1
	* @param param2    Command parameter 2
	*/
    bool publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0){

        if(vehicle_status_listener_->get_Paused_from_Extern_msg()){
            return false;
        }

        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;
        msg.command = command;

        // the target_system field makes problems with mutiple drones... set to 0 to ignore
        // related to mavlink system id
        msg.target_system = 0;
        msg.target_component = 1;

        msg.source_system = 50; //1 is px4 flightcontroller || 255 == qgroundcontrol
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_publisher_->publish(msg);
        return true;
    }


    double publish_circle_poi(double radius, double speed, double latitude, double longitude, double height) override {
        //yaw behvaior 0 center facing, 1 is hold , 2 circle facing , 3 reverse circle facing
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_ORBIT, radius, speed, 0, 0, latitude, longitude, height);
    }

private:
    std::string name_prefix_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener_;
	std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener_;
	std::shared_ptr<GeodeticConverter> gps_converter_;
};

#endif