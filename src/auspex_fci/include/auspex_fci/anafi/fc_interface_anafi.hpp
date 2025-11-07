#ifndef FC_INTERFACE_ANAFI
#define FC_INTERFACE_ANAFI

#include "auspex_fci/fc_interface_base.hpp"
#include "auspex_fci/status_listener_base.hpp"
#include "auspex_fci/position_listener_base.hpp"

// Forward declaration to avoid circular dependency
class ExternalDataListener;


class FC_Interface_ANAFI : public FC_Interface_Base {
public:
    // Use code from cpp file to fill these parts...
    FC_Interface_ANAFI(std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener, std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener, std::string name_prefix): FC_Interface_Base(name_prefix + "_" + "fc_interface_anafi"){
        this->name_prefix_ = name_prefix;
        vehicle_status_listener_ = vehicle_status_listener;
        position_listener_ = position_listener;
        is_initialized_ = true;
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
    }

    void set_height_delta(double height_delta) override {
        // Not needed for ANAFI
    }

    /**
     * @brief Initialize the safety guard with external data listener
     * @param external_data_listener Pointer to external data listener
     */
    void initialize_safety_guard(std::shared_ptr<ExternalDataListener> external_data_listener) override {
        // For ANAFI, we don't implement safety guard yet, but we need to satisfy the interface
        external_data_listener_ = external_data_listener;
        RCLCPP_INFO(this->get_logger(), "SafetyGuard initialization skipped for ANAFI platform");
    }

    /**
	* @brief Send a command to set the home position of the vehicle (px4)
	*/
    void set_home_fc(double lat_deg, double lon_deg, float alt_amsl_m) override {
        RCLCPP_INFO(this->get_logger(), "SetHome command send");
    }

    void terminate() override {
        RCLCPP_INFO(this->get_logger(), "Terminate command not implemented for Anafi");
    }

    void kill() override {
        RCLCPP_INFO(this->get_logger(), "Kill command not implemented for Anafi");
    }

    /**
    * @brief Send a command to the vehicle to take off 10 meters <-- altitude is not relative to the ground. Return true if flying
    */
    bool takeoff(double takeoff_height) override {
        return true;
    }

    /**
    * @brief Send a command to the vehicle to land. Return false if landed successful
    */
    bool land(double latitude, double longitude, double altitude) override {
        return false;
    }

    /**
    * @brief Send a command to Arm the vehicle
    */
    void arm() override {
	    RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    /**
    * @brief Send a command to Disarm the vehicle
    */
    void disarm() override {
        RCLCPP_INFO(this->get_logger(), "Disarm command send");
    }

    /**
     * @brief Set flight variables like speed
     */
    void set_flight_vars(double speed_m_s) override {
        
    }

    /**
     * @brief Move the vehicle to a specific angle
     */
    double move_to_angle(double roll, double pitch, double yaw, double speed_rads=0.1) override{
        
    }

    /**
    * @brief Publish a trajectory setpoint to a gps coordinate returns the distance left to the target while moving towards the target, dont decrease the distances, the target NED trajectory point should stay constant
    */
    double move_to_gps(double latitude, double longitude, double altitude, HEADING heading, double speed_ms=3.0) override {
        return 0.0;
    }

    /**
    * @brief Publish a trajectory setpoint to a ned coordinate
    */
    double move_to_ned(double north, double east, double down, double roll, double pitch, double yaw) override {
        return 0.0;
    }



    void publish_circle_poi(double radius, double speed, double latitude, double longitude, double height) override {
        return;
    }

    /**
    * @brief Send a command to enable position control (PX4)
    */
    bool publish_position_control_mode() override {
        // Not needed
    }

    /**
    * @brief Set FC to offboard control mode.
    */
    bool publish_offboard_control_mode() override {
        // Not needed
    }

    /**
    * @brief Send a heartbeat (PX4) and position control. (PX4)
    */
    void publish_offboard_heartbeat() override {
       // Not needed
    }

    virtual void set_servo_value(int servo_number, int pwm_value) override {
        RCLCPP_INFO(this->get_logger(), "Set servo command not implemented for Anafi");
    }

private:
    std::shared_ptr<ExternalDataListener> external_data_listener_;
};

#endif