#ifndef FLIGHTCONTROLLER_INTERFACE_HPP
#define FLIGHTCONTROLLER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geodetic_converter.hpp"
#include "auspex_fci/status_listener_base.hpp"
#include "auspex_fci/position_listener_base.hpp"


enum HEADING { WAYPOINT = 1, LEFT_90 = 2, RIGHT_90 = 3, UNCHANGED = 4};

class FC_Interface_Base : public rclcpp::Node{
public:

    FC_Interface_Base(const std::string &node_name) : rclcpp::Node(node_name) {
    }


    virtual ~FC_Interface_Base() = default; // Virtual destructor (best practice for base classes)

    /**
     * @brief To get the gps converter. Must be here because one gps message needs to be send to initaliaze.
     */
    virtual void set_gps_converter(std::shared_ptr<GeodeticConverter> gps_converter) = 0;

    /**
	* @brief Send a command to set the home position of the vehicle (px4)
	*/
    virtual void set_home_fc(double lat_deg, double lon_deg, float alt_amsl_m)= 0;

    /**
    * @brief Send a command to the vehicle to terminate. Terminate is shutdown routine.
    */
    virtual void terminate() = 0;

    /**
    * @brief Send a command to the vehicle to kill. Kill is motor stop.
    */
    virtual void kill() = 0;

    /**
    * @brief Send a command to the vehicle to take off 10 meters <-- altitude is not relative to the ground
    */
    virtual bool takeoff(double takeoff_height) = 0;

    /**
    * @brief Send a command to the vehicle to land
    */
    virtual bool land(double latitude, double longitude, double altitude) = 0;

    /**
    * @brief Send a command to Arm the vehicle
    */
    virtual void arm() = 0;

    /**
    * @brief Send a command to Disarm the vehicle
    */
    virtual void disarm() = 0;

    /**
    * @brief Send a command to set position control mode (PX4)
    */
    virtual bool publish_position_control_mode() = 0;

    /**
    * @brief Send a command to set offboard control mode (PX4)
    */
    virtual bool publish_offboard_control_mode() = 0;

    /**
    * @brief Send a heartbeat (PX4) and position control. (PX4)
    */
    virtual void publish_offboard_heartbeat() = 0;

    /**
    * @brief Publish a trajectory setpoint to a gps coordinate returns the distance left to the target while moving towards the target, dont decrease the distances, the target NED trajectory point should stay constant
    */
    virtual double move_to_gps(double latitude, double longitude, double altitude, HEADING heading, double speed_ms=3.0) = 0;

    /**
    * @brief Publish a trajectory setpoint to a ned coordinate
    */
    virtual double move_to_ned(double north, double east, double down, double roll, double pitch, double yaw) = 0;

    /**
    * @brief Publish a command to circle a poi.
    */
    virtual void publish_circle_poi(double radius, double espeedast, double latitude, double longitude, double height) = 0;

    /**
     * @brief Get the VehicleStatusListener object
     * @return std::shared_ptr<VehicleStatusListener_Base>
     */
    std::shared_ptr<VehicleStatusListener_Base> get_vehicle_status_listener() const {
        return vehicle_status_listener_;
    }

    /**
     * @brief Set the VehicleStatusListener object
     * @param vehicle_status_listener std::shared_ptr<VehicleStatusListener_Base>
     */
    void set_vehicle_status_listener(std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener) {
        vehicle_status_listener_ = vehicle_status_listener;
    }

    /**
     * @brief Get the VehicleGlobalPositionListener object
     * @return std::shared_ptr<VehicleGlobalPositionListener_Base>
     */
    std::shared_ptr<VehicleGlobalPositionListener_Base> get_position_listener() const {
        return position_listener_;
    }

    /**
     * @brief Set the VehicleGlobalPositionListener object
     * @param position_listener std::shared_ptr<VehicleGlobalPositionListener_Base>
     */
    void set_position_listener(std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener) {
        position_listener_ = position_listener;
    }

    bool get_is_initialized() const {
        return is_initialized_;
    }

protected:
    bool is_initialized_ = false;
    std::string name_prefix_;

    std::shared_ptr<GeodeticConverter> gps_converter_; // Geodetic converter for GPS coordinates
    std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener_; // Vehicle status listener
    std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener_; // Position listener

};

#endif