#ifndef FC_INTERFACE_MAVSDK
#define FC_INTERFACE_MAVSDK

#include "auspex_fci/fc_interface_base.hpp"
#include "auspex_fci/safety_guard.hpp"
#include "auspex_fci/status_listener_base.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include "auspex_fci/mavsdk/position_listener_mavsdk.hpp"
#include "auspex_fci/mavsdk/status_listener_mavsdk.hpp"

// Forward declaration
class ExternalDataListener;

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugin_base.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/log_callback.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct ConnectionSettings {
    std::string connection_type;
    std::string ip_path;
    std::string port_baud;
};

class FC_Interface_MAVSDK : public FC_Interface_Base {
public:

    FC_Interface_MAVSDK(std::string name_prefix, std::string FC_TYPE)
        : FC_Interface_Base(name_prefix + "_" + "fci_mav"),
        mavsdk(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer})
    {   

        mavsdk::log::subscribe([](mavsdk::log::Level level,   // message severity level
                          const std::string& message, // message text
                          const std::string& file,    // source file from which the message was sent
                          int line) {                 // line number in the source file
            return level < mavsdk::log::Level::Warn;
        });

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

        if (connection_result != mavsdk::ConnectionResult::Success) {
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
        passthrough_ = std::make_shared<mavsdk::MavlinkPassthrough>(system_);

        vehicle_status_listener_ = std::make_shared<VehicleStatusListener_MAVSDK>(name_prefix, system_, FC_TYPE);
        position_listener_ = std::make_shared<VehicleGlobalPositionListener_MAVSDK>(name_prefix, system_, FC_TYPE);

        action_ = std::make_shared<mavsdk::Action>(system_);
        offboard_ = std::make_shared<mavsdk::Offboard>(system_);
        param_ = std::make_shared<mavsdk::Param>(system_);

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
     * @brief Sets the external data listener for collision avoidance
     */
    void set_external_data_listener(std::shared_ptr<ExternalDataListener> external_data_listener) override {
        external_data_listener_ = external_data_listener;
        if (external_data_listener_ && !safety_guard_) {
            initialize_safety_guard(external_data_listener_);
        }
    }

    void set_height_delta(double height_delta) override {
        height_delta_ = height_delta;
    }

    /**
     * @brief Initialize the safety guard with external data listener
     * @param external_data_listener Pointer to external data listener
     */
    void initialize_safety_guard(std::shared_ptr<ExternalDataListener> external_data_listener) override {
        if (!safety_guard_ && external_data_listener) {
            safety_guard_ = std::make_shared<SafetyGuard>(name_prefix_, external_data_listener, this->get_logger());
            RCLCPP_INFO(this->get_logger(), "SafetyGuard initialized for platform: %s", name_prefix_.c_str());
        }
    }

    /**
	* @brief Send a command to set the home position of the vehicle
	*/
    void set_home_fc(double lat_deg, double lon_deg, float alt_amsl_m) override {
        mavsdk::MavlinkPassthrough::CommandLong cmd{};
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
        action_->terminate_async([](mavsdk::Action::Result result) {
            // Empty callback, do nothing with result
        });
        RCLCPP_INFO(this->get_logger(), "Terminate command sent");
    }

    void kill() override {
        action_->kill_async([](mavsdk::Action::Result result) {
            // Empty callback, do nothing with result
        });
        RCLCPP_INFO(this->get_logger(), "Kill command sent");
    }

    /**
    * @brief Send a command to Arm the vehicle
    */
    void arm() override {
        if((position_listener_->get_recent_home_msg()->latitude_deg <= 0.0 ||
           position_listener_->get_recent_home_msg()->longitude_deg <= 0.0 ||
           position_listener_->get_recent_home_msg()->absolute_altitude_m <= 0.0)) {
            RCLCPP_ERROR(this->get_logger(), "Home position not set! Refusing to arm.");
            return;
        }
        const mavsdk::Action::Result arm_result = action_->arm();
        if (arm_result != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Arming failed: %d", static_cast<int>(arm_result));
            return;
        }
	    RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    /**
    * @brief Send a command to Disarm the vehicle
    */
    void disarm() override {
        const mavsdk::Action::Result disarm_result = action_->disarm();
        if (disarm_result != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Disarming failed: %d", static_cast<int>(disarm_result));
            return;
        }
	    RCLCPP_INFO(this->get_logger(), "Disarm command send");
    }

    /**
    * @brief Send a command to the vehicle to take off 10 meters <-- altitude is not relative to the ground
    */
    bool takeoff(double takeoff_height) override {
        action_->set_takeoff_altitude(static_cast<float>(takeoff_height + height_delta_));
        const mavsdk::Action::Result takeoff_result = action_->takeoff();
        if (takeoff_result != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed: %d", static_cast<int>(takeoff_result));
            return false;
        }
        return true;
    }

    /**
    * @brief Send a command to the vehicle to land
    */
    bool land(double latitude, double longitude, double altitude) override {
        const mavsdk::Action::Result land_result = action_->land();
        if (land_result != mavsdk::Action::Result::Success) {
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
            
            // Get current yaw instead of using NaN which causes issues with ArduPilot
            if (position_listener_ && position_listener_->get_recent_ned_msg()) {
                tf2::Quaternion current_q(
                    position_listener_->get_recent_ned_msg()->q.x,
                    position_listener_->get_recent_ned_msg()->q.y,
                    position_listener_->get_recent_ned_msg()->q.z,
                    position_listener_->get_recent_ned_msg()->q.w
                );
                tf2::Matrix3x3 m(current_q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                noop_sp.yaw_deg = static_cast<float>(yaw * 180.0 / M_PI);
            } else {
                noop_sp.yaw_deg = 0.0f;  // Default to 0 if no position data available
            }
            
            auto set_res = offboard_->set_velocity_ned(noop_sp);

            if (set_res != mavsdk::Offboard::Result::Success) {
                RCLCPP_ERROR(get_logger(), "Failed to send initial no-op setpoint: %d", static_cast<int>(set_res));
                return false;
            }
            mavsdk::Offboard::Result offboard_result = offboard_->start();
            if (offboard_result != mavsdk::Offboard::Result::Success) {
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
     * @brief Move the vehicle to a specific angle while maintaining current position
     */
    double move_to_angle(double roll, double pitch, double yaw, double speed_rads=0.1) override{
        // Get current position (keep position unchanged)
        double current_north = position_listener_->get_recent_ned_msg()->position_body.x_m;
        double current_east = position_listener_->get_recent_ned_msg()->position_body.y_m;
        double current_down = position_listener_->get_recent_ned_msg()->position_body.z_m;
        
        // Get current yaw to calculate remaining deviation
        tf2::Quaternion current_q(
            position_listener_->get_recent_ned_msg()->q.x,
            position_listener_->get_recent_ned_msg()->q.y,
            position_listener_->get_recent_ned_msg()->q.z,
            position_listener_->get_recent_ned_msg()->q.w
        );
        tf2::Matrix3x3 m(current_q);
        double current_roll, current_pitch, current_yaw;
        m.getRPY(current_roll, current_pitch, current_yaw);
        
        // Calculate yaw deviation (shortest angular distance)
        double yaw_deviation = yaw - current_yaw;
        // Normalize to [-PI, PI]
        while (yaw_deviation > M_PI) yaw_deviation -= 2.0 * M_PI;
        while (yaw_deviation < -M_PI) yaw_deviation += 2.0 * M_PI;
        
        if(!vehicle_status_listener_->get_paused_from_extern()){
            // Create NED position setpoint with new orientation
            mavsdk::Offboard::PositionNedYaw setpoint{};
            setpoint.north_m = static_cast<float>(current_north);
            setpoint.east_m = static_cast<float>(current_east);
            setpoint.down_m = static_cast<float>(current_down);
            setpoint.yaw_deg = static_cast<float>(yaw * 180.0 / M_PI);

            this->set_flight_vars(1.5);
            offboard_->set_position_ned(setpoint);
        }
        
        return fabs(yaw_deviation);
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

        double target_yaw;
        if(heading == HEADING::WAYPOINT){
            // Calculate bearing from current GPS position to target GPS position
            double current_lat = position_listener_->get_recent_gps_msg()->latitude_deg;
            double current_lon = position_listener_->get_recent_gps_msg()->longitude_deg;
            
            // Convert degrees to radians for calculation
            double lat1_rad = current_lat * M_PI / 180.0;
            double lat2_rad = latitude * M_PI / 180.0;
            double delta_lon_rad = (longitude - current_lon) * M_PI / 180.0;
            
            // Calculate bearing using great circle navigation
            double x = sin(delta_lon_rad) * cos(lat2_rad);
            double y = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon_rad);
            target_yaw = atan2(x, y); // [-PI:PI] - bearing from north
            target_yaw = gps_converter_->discretize(target_yaw, 5.0);
        }else if(heading == HEADING::UNCHANGED){
            tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q.x, position_listener_->get_recent_ned_msg()->q.y, position_listener_->get_recent_ned_msg()->q.z, position_listener_->get_recent_ned_msg()->q.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, target_yaw);
        }else{
            // Default case: extract current yaw to maintain current heading
            RCLCPP_WARN(this->get_logger(), "Unknown HEADING enum value, maintaining current orientation");
            tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q.x, position_listener_->get_recent_ned_msg()->q.y, position_listener_->get_recent_ned_msg()->q.z, position_listener_->get_recent_ned_msg()->q.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, target_yaw);
        }

        if(!vehicle_status_listener_->get_paused_from_extern()){
            // Safety check: Perform collision avoidance before setting position
            if (safety_guard_) {
                // Create current and target GPS positions for safety check
                geographic_msgs::msg::GeoPoint current_gps;
                current_gps.latitude = position_listener_->get_recent_gps_msg()->latitude_deg;
                current_gps.longitude = position_listener_->get_recent_gps_msg()->longitude_deg;
                current_gps.altitude = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
                
                geographic_msgs::msg::GeoPoint target_gps;
                target_gps.latitude = latitude;
                target_gps.longitude = longitude;
                target_gps.altitude = altitude;
                
                // Create current pose for safety check
                geometry_msgs::msg::Pose current_pose;
                current_pose.position.x = current_north;
                current_pose.position.y = current_east;
                current_pose.position.z = current_down;
                current_pose.orientation.x = position_listener_->get_recent_ned_msg()->q.x;
                current_pose.orientation.y = position_listener_->get_recent_ned_msg()->q.y;
                current_pose.orientation.z = position_listener_->get_recent_ned_msg()->q.z;
                current_pose.orientation.w = position_listener_->get_recent_ned_msg()->q.w;
                
                // Check if movement is safe
                if (!safety_guard_->isSafeMovementGPS(current_gps, target_gps, current_pose)) {
                    // Collision detected - hover at current position
                    RCLCPP_WARN(this->get_logger(), "Collision detected! Hovering at current position");
                    
                    // For now, just hover at current position - safe hover calculation can be added later
                    mavsdk::Offboard::PositionGlobalYaw hover_setpoint{};
                    hover_setpoint.lat_deg = current_gps.latitude;
                    hover_setpoint.lon_deg = current_gps.longitude;
                    hover_setpoint.alt_m = current_gps.altitude;
                    hover_setpoint.altitude_type = mavsdk::Offboard::PositionGlobalYaw::AltitudeType::Amsl;
                    hover_setpoint.yaw_deg = target_yaw * 180.0 / M_PI;
                    
                    offboard_->set_position_global(hover_setpoint);
                    return gps_converter_->getDistance(delta_n, delta_e, delta_d); // Return original distance
                }
            }
            
            mavsdk::Offboard::PositionGlobalYaw setpoint{};
            setpoint.lat_deg = latitude;
            setpoint.lon_deg = longitude;
            setpoint.alt_m = static_cast<float>(altitude + height_delta_);
            setpoint.altitude_type = mavsdk::Offboard::PositionGlobalYaw::AltitudeType::Amsl;
            setpoint.yaw_deg = static_cast<float>(target_yaw * 180.0 / M_PI);

            this->set_flight_vars(speed_m_s);
            offboard_->set_position_global(setpoint);
        }

        return gps_converter_->geodeticDistance2D(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, latitude, longitude);
    }

    void set_flight_vars(double speed_m_s) override {
        float turn_speed_degree_s = 30.0;
        float turn_acc_degree_s2 = 5.0;

        float ardu_turn_speed_degree_s = 30.0;
        float ardu_turn_acc_cdegree_s2 = 500.0;

        
        if(current_speed_m_s != speed_m_s){
            RCLCPP_INFO(this->get_logger(), "Setting flight speed = %.2f m/s", speed_m_s);
            if (FC_TYPE_ == "ARDUPILOT") {
                float speed_cm_s = speed_m_s * 100.0f;
                //param_->set_param_float("WPNAV_SPEED", speed_cm_s);
                //param_->set_param_float("ATC_RATE_Y_MAX", static_cast<float>(ardu_turn_speed_degree_s));
                //param_->set_param_float("ATC_ACCEL_Y_MAX", static_cast<float>(ardu_turn_acc_cdegree_s2));
                //param_->set_param_float("ATC_SLEW_YAW", 500.0f); //cdeg/s
            } else {
                param_->set_param_float("MPC_XY_VEL_MAX", static_cast<float>(speed_m_s));
                param_->set_param_float("MPC_YAWRAUTO_ACC", turn_speed_degree_s);
                param_->set_param_float("MPC_YAWRAUTO_MAX", turn_acc_degree_s2);
            }
            current_speed_m_s = speed_m_s;
        }
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

        mavsdk::Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = north;
        setpoint.east_m = east;
        setpoint.down_m = down;
        setpoint.yaw_deg = yaw * 180.0 / M_PI;

        if(!vehicle_status_listener_->get_paused_from_extern()){
            // Safety check: Perform collision avoidance before setting position
            if (safety_guard_) {
                // Create current pose for safety check
                geometry_msgs::msg::Pose current_pose;
                current_pose.position.x = current_north;
                current_pose.position.y = current_east;
                current_pose.position.z = current_down;
                current_pose.orientation.x = position_listener_->get_recent_ned_msg()->q.x;
                current_pose.orientation.y = position_listener_->get_recent_ned_msg()->q.y;
                current_pose.orientation.z = position_listener_->get_recent_ned_msg()->q.z;
                current_pose.orientation.w = position_listener_->get_recent_ned_msg()->q.w;
                
                // Create target position for safety check
                geometry_msgs::msg::Point target_position;
                target_position.x = north;
                target_position.y = east;
                target_position.z = down;
                
                geometry_msgs::msg::Point current_position;
                current_position.x = current_north;
                current_position.y = current_east;
                current_position.z = current_down;
                
                // Check if NED movement is safe
                if (!safety_guard_->isSafeMovementNED(current_position, target_position)) {
                    // Collision detected - hover at current position
                    RCLCPP_WARN(this->get_logger(), "NED collision detected! Hovering at current position");
                    
                    mavsdk::Offboard::PositionNedYaw hover_setpoint{};
                    hover_setpoint.north_m = static_cast<float>(current_north);
                    hover_setpoint.east_m = static_cast<float>(current_east);
                    hover_setpoint.down_m = static_cast<float>(current_down);
                    hover_setpoint.yaw_deg = static_cast<float>(yaw * 180.0 / M_PI);
                    
                    offboard_->set_position_ned(hover_setpoint);
                    return gps_converter_->getDistance(delta_n, delta_e, delta_d); // Return original distance
                }
            }
            
            offboard_->set_position_ned(setpoint);
        }

        return gps_converter_->getDistance(delta_n, delta_e, delta_d);
    }

    void publish_circle_poi(double radius_m, double speed_ms, double latitude, double longitude, double height) override {
        auto result = action_->do_orbit(
            static_cast<float>(radius_m),
            static_cast<float>(speed_ms),
            mavsdk::Action::OrbitYawBehavior::HoldFrontToCircleCenter,
            latitude,
            longitude,
            height + height_delta_
        );
        if (result != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Orbit command failed: %d", static_cast<int>(result));
            return;
        }
        return;
    }

    void set_servo_value(int servo_number, int pwm_value) override {
        if (servo_number < 1 || servo_number > 16) {
            RCLCPP_ERROR(this->get_logger(), "Servo number %d out of range (1-16)", servo_number);
            return;
        }
        
        if(FC_TYPE_ == "ARDUPILOT") {
            RCLCPP_ERROR(this->get_logger(), "Set servo command only implemented for PX4");
            return;
        }else{
            // Clamp PWM values to discrete ranges
            if (pwm_value < 1250) {
                pwm_value = 1000;
            } else if (pwm_value <= 1750) {
                pwm_value = 1500;
            } else {
                pwm_value = 2000;
            }
            
            float normalized_value = (pwm_value - 1500.0f) / 500.0f;

            mavsdk::MavlinkPassthrough::CommandLong cmd{};
            cmd.target_sysid  = passthrough_->get_target_sysid();
            cmd.target_compid = passthrough_->get_target_compid();
            cmd.command       = MAV_CMD_DO_SET_ACTUATOR;
            cmd.param1        = static_cast<float>(normalized_value); 
            cmd.param2        = NAN; // Unused
            cmd.param3        = NAN; // Unused
            cmd.param4        = NAN; // Unused
            cmd.param5        = NAN; // Unused
            cmd.param6        = NAN; // Unused
            cmd.param7        = NAN; // Unused

            passthrough_->send_command_long(cmd);
            RCLCPP_INFO(this->get_logger(), "Set servo %d to PWM %d", servo_number, pwm_value);
        }
    }

private:
    double                           current_speed_m_s = -1.0;
    double                           height_delta_ = 0.0;
    mavsdk::Mavsdk                   mavsdk;
    std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough_{nullptr};
    std::shared_ptr<mavsdk::System>          system_{nullptr};
    std::shared_ptr<mavsdk::Action>          action_;
    std::shared_ptr<mavsdk::Offboard>        offboard_;
    std::shared_ptr<mavsdk::Param>           param_;
    std::shared_ptr<ExternalDataListener> external_data_listener_{nullptr};
};

#endif