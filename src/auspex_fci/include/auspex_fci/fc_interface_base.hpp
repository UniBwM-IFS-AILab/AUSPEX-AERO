#ifndef FLIGHTCONTROLLER_INTERFACE_HPP
#define FLIGHTCONTROLLER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include "geodetic_converter.hpp"


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
    virtual void set_home_fc(double latitude, double longitude, double altitude) = 0;

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
    * @brief Send a command to enable position control (PX4)
    */
    virtual bool publish_position_control_mode() = 0;

    /**
    * @brief Set FC to offboard control mode. Should return true if in offboard mode and fals if not.
    */
    virtual bool publish_offboard_control_mode() = 0;

    /**
    * @brief Send a heartbeat (PX4) and position control. (PX4)
    */
    virtual void publish_offboard_heartbeat() = 0;

    /**
    * @brief Send a heartbeat (PX4) and position and vel control. (PX4)
    */
    virtual void publish_offboard_heartbeatVel() = 0;

    /**
    * @brief Publish a command to let the vehicle hover in a given ned position.
    */
    virtual void hover_in_position(double hover_north, double hover_east, double hover_down, double roll, double pitch, double yaw) = 0;

    /**
    * @brief Publish a command to let the vehicle hover in the current position.
    */
    virtual void hover_in_current_position() = 0;

    /**
    * @brief Publish a trajectory setpoint to a gps coordinate returns the distance left to the target while moving towards the target, dont decrease the distances, the target NED trajectory point should stay constant
    */
    virtual double move_to_gps(double latitude, double longitude, double altitude, HEADING heading) = 0;

    /**
    * @brief Publish a trajectory setpoint to a ned coordinate
    */
    virtual double move_to_ned(double north, double east, double down, double roll, double pitch, double yaw) = 0;

    /**
    * @brief Publish a command to circle a poi.
    */
    virtual double publish_circle_poi(double radius, double espeedast, double latitude, double longitude, double height) = 0;
};

#endif