#ifndef FC_INTERFACE_MAVSDK
#define FC_INTERFACE_MAVSDK

#include "auspex_fci/fc_interface_base.hpp"
#include "auspex_fci/status_listener_base.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include "auspex_fci/mavsdk/position_listener_mavsdk.hpp"
#include "auspex_fci/mavsdk/status_listener_mavsdk.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugin_base.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct ConnectionSettings {
    std::string connection_type;
    std::string ip_path;
    std::string port_baud;
};

using namespace mavsdk;

class FC_Interface_MAVSDK : public FC_Interface_Base {
public:

    FC_Interface_MAVSDK(std::string name_prefix, std::string FC_TYPE)
        : FC_Interface_Base(name_prefix + "_" + "fc_interface_mavsdk"),
        mavsdk(Mavsdk::Configuration{ComponentType::CompanionComputer})
    {
        FC_TYPE_ = FC_TYPE;

        this->name_prefix_ = name_prefix;

        std::map<std::string, ConnectionSettings> connection_profiles = {
            {"DESKTOP", {"udpin","127.0.0.1", "14540"}},
            {"ARDUPILOT",  {"serial", "/dev/ttyTHS0", "57600"}},
            {"PX4",      {"serial", "/dev/ttyAMA0", "921600"}}
        };

        std::string selected_type = FC_TYPE;
        if (FC_TYPE.find("SIMULATED") != std::string::npos) {
            selected_type = "DESKTOP";
        }

        auto it = connection_profiles.find(selected_type);
        if (it == connection_profiles.end()) {
            throw std::runtime_error("Unknown FC_TYPE: " + FC_TYPE);
            return;
        }
        auto settings = it->second;

        if (selected_type == "DESKTOP") {
            size_t underscore_pos = name_prefix.find_last_of('_');
            if (underscore_pos != std::string::npos) {
                std::string vehicle_num_str = name_prefix.substr(underscore_pos + 1);
                try {
                    int vehicle_num = std::stoi(vehicle_num_str);
                    if (vehicle_num >= 10) {
                        throw std::runtime_error("Vehicle number " + std::to_string(vehicle_num) + " exceeds maximum of 9 vehicles");
                        return;
                    }
                    int base_port = std::stoi(settings.port_baud);
                    settings.port_baud = std::to_string(base_port + vehicle_num);
                } catch (const std::invalid_argument& e) {
                    RCLCPP_WARN(get_logger(), "Could not parse vehicle number from prefix '%s', using default port", name_prefix.c_str());
                }
            }
        }

        auto [connection_result, connection_handle] =
            mavsdk.add_any_connection_with_handle(settings.connection_type + "://" + settings.ip_path + ":" + settings.port_baud);

        if (connection_result != ConnectionResult::Success) {
            RCLCPP_ERROR(get_logger(), "Failed to bind %s input on port %s", settings.connection_type.c_str(), settings.port_baud.c_str());
            mavsdk.remove_connection(connection_handle);
            return;
        }

        auto maybe_system = mavsdk.first_autopilot(10.0);
        if (!maybe_system.has_value()) {
            std::cerr << "No autopilot found within timeout\n";
            mavsdk.remove_connection(connection_handle);
            return;
        }
        system_ = maybe_system.value();
        passthrough_ = std::make_shared<MavlinkPassthrough>(system_);

        vehicle_status_listener_ = std::make_shared<VehicleStatusListener_MAVSDK>(name_prefix, system_, FC_TYPE);
        position_listener_ = std::make_shared<VehicleGlobalPositionListener_MAVSDK>(name_prefix, system_, FC_TYPE);


        action_ = std::make_shared<Action>(system_);
        offboard_ = std::make_shared<Offboard>(system_);
        param_ = std::make_shared<Param>(system_);

        is_initialized_ = true;
    }

    /**
     * @brief Destructor - properly cleanup MAVSDK resources
     */
    ~FC_Interface_MAVSDK() {
        if (offboard_ && offboard_->is_active()) {
            offboard_->stop();
        }
        
        // Reset shared pointers to ensure proper cleanup order
        action_.reset();
        offboard_.reset();
        param_.reset();
        passthrough_.reset();
        
        // Reset listeners
        vehicle_status_listener_.reset();
        position_listener_.reset();
        
        // Reset system pointer
        system_.reset();
        
        RCLCPP_INFO(get_logger(), "FC_Interface_MAVSDK destructor completed");
    }

    /**
     * @brief To get the gps converter. Must be here because one gps message needs to be send to initaliaze.
     */
    void set_gps_converter(std::shared_ptr<GeodeticConverter> gps_converter) override {
        gps_converter_ = gps_converter;
    }

    /**
	* @brief Send a command to set the home position of the vehicle
	*/
    void set_home_fc(double lat_deg, double lon_deg, float alt_amsl_m) override {
        MavlinkPassthrough::CommandLong cmd{};
        cmd.target_sysid  = passthrough_->get_target_sysid();
        cmd.target_compid = passthrough_->get_target_compid();
        cmd.command       = MAV_CMD_DO_SET_HOME;
        cmd.param1        = 0;
        cmd.param5        = static_cast<float>(lat_deg);
        cmd.param6        = static_cast<float>(lon_deg);
        cmd.param7        = alt_amsl_m;

        passthrough_->send_command_long(cmd);
    }

    void terminate() override {
        action_->terminate_async([](Action::Result result) {
            // Empty callback, do nothing with result
        });
        RCLCPP_INFO(this->get_logger(), "Terminate command sent");
    }

    void kill() override {
        action_->kill_async([](Action::Result result) {
            // Empty callback, do nothing with result
        });
        RCLCPP_INFO(this->get_logger(), "Kill command sent");
    }

    /**
    * @brief Send a command to Arm the vehicle
    */
    void arm() override {
        const Action::Result arm_result = action_->arm();
        if (arm_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Arming failed: %d", static_cast<int>(arm_result));
            return;
        }
	    RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    /**
    * @brief Send a command to Disarm the vehicle
    */
    void disarm() override {
        const Action::Result disarm_result = action_->disarm();
        if (disarm_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Disarming failed: %d", static_cast<int>(disarm_result));
            return;
        }
	    RCLCPP_INFO(this->get_logger(), "Disarm command send");
    }

    /**
    * @brief Send a command to the vehicle to take off 10 meters <-- altitude is not relative to the ground
    */
    bool takeoff(double takeoff_height) override {
        action_->set_takeoff_altitude(takeoff_height);
        const Action::Result takeoff_result = action_->takeoff();
        if (takeoff_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed: %d", static_cast<int>(takeoff_result));
            return false;
        }
        return true;
    }

    /**
    * @brief Send a command to the vehicle to land
    */
    bool land(double latitude, double longitude, double altitude) override {
        const Action::Result land_result = action_->land();
        if (land_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Land failed: %d", static_cast<int>(land_result));
            return false;
        }
        return true;
    }

    /**
    * @brief Send a command to enable position control (PX4)
    */
    bool publish_position_control_mode() override {
        auto result = action_->hold();
        if (result != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(get_logger(),"Hold (position hold) failed: %d",static_cast<int>(result));
        }
    }

    /**
    * @brief Set FC to offboard control mode.
    */
    bool publish_offboard_control_mode() override {
        if (!offboard_->is_active()) {
            mavsdk::Offboard::VelocityNedYaw noop_sp{};
            noop_sp.north_m_s = 0.0f;
            noop_sp.east_m_s  = 0.0f;
            noop_sp.down_m_s  = 0.0f;     // zero down = hold current altitude
            noop_sp.yaw_deg   = std::nanf("");
            auto set_res = offboard_->set_velocity_ned(noop_sp);

            if (set_res != mavsdk::Offboard::Result::Success) {
                RCLCPP_ERROR(get_logger(),
                            "Failed to send initial no-op setpoint: %d",
                            static_cast<int>(set_res));
                return false;
            }
            Offboard::Result offboard_result = offboard_->start();
            if (offboard_result != Offboard::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Offboard start failed: %d", static_cast<int>(offboard_result));
                return false;
            }
        }
        return true;
    }

    /**
    * @brief Send a heartbeat and position control. Only needed for uxrce -> mavsdk does this automatically
    */
    void publish_offboard_heartbeat() override {
    }

    /**
    * @brief Publish a trajectory setpoint to a gps coordinate returns the distance left to the target while moving towards the target, dont decrease the distances, the target NED trajectory point should stay constant
    */
    double move_to_gps(double latitude, double longitude, double altitude, HEADING heading, double speed_m_s = 1.5) override {
        // NED Coordinates for target
        double north, east, down;
        gps_converter_->geodetic2Ned(latitude, longitude, altitude, &north, &east, &down); // alt in above mean sea level

        // receive current NED
        double current_north = position_listener_->get_recent_ned_msg()->position_body.x_m;
        double current_east = position_listener_->get_recent_ned_msg()->position_body.y_m;
        double current_down = position_listener_->get_recent_ned_msg()->position_body.z_m;
        double delta_n = north - current_north, delta_e = east - current_east, delta_d = down - current_down;

        delta_n = gps_converter_->discretize(delta_n,5.0);
        delta_e = gps_converter_->discretize(delta_e,5.0);
        delta_d = gps_converter_->discretize(delta_d,5.0);
        north = gps_converter_->discretize(north,5.0);
        east = gps_converter_->discretize(east,5.0);
        down = gps_converter_->discretize(down,5.0);

        double target_yaw = 0.0;
        if(heading == HEADING::WAYPOINT){
            target_yaw = atan2(east -current_east,north - current_north); // [-PI:PI]
            target_yaw = gps_converter_->discretize(target_yaw,5.0);
        }else if(heading == HEADING::UNCHANGED){
            tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q.x,position_listener_->get_recent_ned_msg()->q.y,position_listener_->get_recent_ned_msg()->q.z,position_listener_->get_recent_ned_msg()->q.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, target_yaw);
        }

        if(!vehicle_status_listener_->get_paused_from_extern()){
            Offboard::PositionGlobalYaw setpoint{};
            setpoint.lat_deg = latitude;
            setpoint.lon_deg = longitude;
            setpoint.alt_m = altitude;
            setpoint.altitude_type = Offboard::PositionGlobalYaw::AltitudeType::Amsl;
            setpoint.yaw_deg = target_yaw * 180.0 / M_PI;

            float speed_degree_s = 5.0;
            float acc_degree_s2 = 5.0;

            int ardu_speed_degree_s = 360;
            int ardu_acc_cdegree_s2 = 9000;

            if(current_speed_m_s != speed_m_s){
                if (FC_TYPE_ == "ARDUPILOT") {
                    int speed_cm_s = static_cast<int>(speed_m_s * 100.0f);
                    param_->set_param_int("WPNAV_SPEED", static_cast<int>(speed_cm_s));
                    param_->set_param_int("ATC_RATE_Y_MAX", ardu_speed_degree_s);
                    param_->set_param_int("ATC_ACCEL_Y_MAX", ardu_acc_cdegree_s2);
                    param_->set_param_int("ATC_SLEW_YAW", 500); //cdeg/s
                } else {
                    param_->set_param_float("MPC_XY_VEL_MAX", static_cast<float>(speed_m_s));
                    param_->set_param_float("MPC_YAWRAUTO_ACC", speed_degree_s);
                    param_->set_param_float("MPC_YAWRAUTO_MAX", acc_degree_s2);
                }
                current_speed_m_s = speed_m_s;
            }
            offboard_->set_position_global(setpoint);
        }
        return gps_converter_->getDistance(delta_n, delta_e, delta_d);
    }

    /**
    * @brief Publish a trajectory setpoint to a ned coordinate
    */
    double move_to_ned(double north, double east, double down, double roll, double pitch, double yaw) override {
        // Calculate distance to target
        double current_north = position_listener_->get_recent_ned_msg()->position_body.x_m;
        double current_east = position_listener_->get_recent_ned_msg()->position_body.y_m;
        double current_down = position_listener_->get_recent_ned_msg()->position_body.z_m;

        double delta_n = north - current_north;
        double delta_e = east - current_east;
        double delta_d = down - current_down;

        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = north;
        setpoint.east_m = east;
        setpoint.down_m = down;
        setpoint.yaw_deg = yaw * 180.0 / M_PI;

        if(!vehicle_status_listener_->get_paused_from_extern()){
            offboard_->set_position_ned(setpoint);
        }

        return gps_converter_->getDistance(delta_n, delta_e, delta_d);
    }

    void publish_circle_poi(double radius_m, double speed_ms, double latitude, double longitude, double height) override {
        auto result = action_->do_orbit(
            static_cast<float>(radius_m),
            static_cast<float>(speed_ms),
            Action::OrbitYawBehavior::HoldFrontToCircleCenter,
            latitude,
            longitude,
            height
        );
        if (result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Orbit command failed: %d", static_cast<int>(result));
            return;
        }
        return;
    }

private:
    double                           current_speed_m_s = -1.0;
    std::string                      FC_TYPE_;
    Mavsdk                           mavsdk;
    std::shared_ptr<MavlinkPassthrough> passthrough_{nullptr};
    std::shared_ptr<System>          system_{nullptr};
    std::shared_ptr<Action>          action_;
    std::shared_ptr<Offboard>        offboard_;
    std::shared_ptr<Param>           param_;
};

#endif