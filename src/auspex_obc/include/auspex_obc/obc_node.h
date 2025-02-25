#ifndef OBC_NODE_H
#define OBC_NODE_H

#include <cstdlib>

//A vector 3d class
#include "auspex_obc/Vector3D.h"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//auspex msgs
#include "auspex_msgs/action/execute_sequence.hpp"

#include "auspex_msgs/msg/object_knowledge.hpp"
#include "auspex_msgs/msg/execute_atom.hpp"
#include "auspex_msgs/msg/altitude_level.hpp"

#include "auspex_msgs/srv/get_origin.hpp"
#include "auspex_msgs/srv/set_origin.hpp"
#include "auspex_msgs/srv/get_altitude.hpp"
#include "auspex_msgs/srv/add_action.hpp"

#include "auspex_msgs/srv/insert_knowledge.hpp"
#include "auspex_msgs/srv/exists_knowledge.hpp"

// Base Class
#include "auspex_fci/fc_interface_base.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include "auspex_fci/status_listener_base.hpp"

// For PX4
#include "auspex_fci/px4/fc_interface_px4.hpp"
#include "auspex_fci/px4/position_listener_px4.hpp"
#include "auspex_fci/px4/status_listener_px4.hpp"

// For ANAFI
#include "auspex_fci/anafi/fc_interface_anafi.hpp"
#include "auspex_fci/anafi/position_listener_anafi.hpp"
#include "auspex_fci/anafi/status_listener_anafi.hpp"

#include "msg_context_cpp/message_loader.hpp"

// following include is for transforming gps to NED coordinates
#include "auspex_fci/geodetic_converter.hpp"

// Nodes for listening to px4 messages
#include "auspex_obc/drone_state_publisher.hpp"

#ifdef INCLUDE_SIM_HEADER
	#include "auspex_obc/cam_sim.hpp"
	typedef SimCamPublisher CamPublisher;
#else
	#include "auspex_obc/cam_rpi5.hpp"
	typedef RPI5_CameraPublisher CamPublisher;
#endif
      
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <mutex>

#include <chrono>
#include <thread>
#include <iostream>
#include <queue>
#include <math.h>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;


// msg
using ObjectKnowledge = auspex_msgs::msg::ObjectKnowledge;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using ExecuteAtom = auspex_msgs::msg::ExecuteAtom;
using AltitudeLevel = auspex_msgs::msg::AltitudeLevel;

// action
using ExecuteSequence = auspex_msgs::action::ExecuteSequence;

// srv
using GetAltitude = auspex_msgs::srv::GetAltitude;
using InsertKnowledge = auspex_msgs::srv::InsertKnowledge;
using ExistsKnowledge = auspex_msgs::srv::ExistsKnowledge;

class OffboardController : public rclcpp::Node {
public:
	
	std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener_;
	std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener_;
	std::shared_ptr<DroneStatePublisher> drone_state_publisher_;
	std::shared_ptr<CamPublisher> camera_publisher_;
	std::shared_ptr<FC_Interface_Base> fc_interface_;

	std::deque<ExecuteAtom> executeSequenceQueue;

	// name_prefix should have the format "<identifier>/"
	OffboardController(std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener, 
					std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener, 
					std::shared_ptr<FC_Interface_Base> fc_interface,
					std::shared_ptr<DroneStatePublisher> drone_state_publisher,
					std::shared_ptr<CamPublisher> cam_publisher,
					std::string name_prefix = "") ;
	
	~OffboardController() = default;
	
	double getHeightAMSL(AltitudeLevel level, double value);
	double get_groundHeight_amsl();
	void timer_callback();
	bool checkForExternalCommand();//returns true if there was and external command
	void start_action_server();
	void start_services();
	void arm();
	void disarm();

private:
	enum RETURN_VALUE { action_critical_failure = -2, action_failure = -1, action_completed = 0, goal_succeeded = 4, goal_canceled = 5, action_paused = 6, action_splitted =7};

	std::string _current_action_type;
	std::mutex execute_queue_lock;
	
	// pointer to action server objects
	rclcpp_action::Server<ExecuteSequence>::SharedPtr sequence_action_server_;
	
	// pointer to services
	rclcpp::Service<auspex_msgs::srv::GetOrigin>::SharedPtr get_origin_service_;
	rclcpp::Service<auspex_msgs::srv::SetOrigin>::SharedPtr set_origin_service_;
	rclcpp::Service<auspex_msgs::srv::AddAction>::SharedPtr handle_add_action_service_;

	rclcpp::Client<GetAltitude>::SharedPtr get_altitude_client;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer_printer;

	std::shared_ptr<GeodeticConverter> gps_converter_;

	rclcpp::Subscription<UserCommand>::SharedPtr handle_user_command_listener_;

	bool is_flying_ = false; //!< boolean for checking if the drone needs to arm and takeoff
	bool is_height_data_available = false;
	std::string name_prefix_;

	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const typename ExecuteSequence::Goal> goal){
		std::cout << "In handle_goal \n";
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
	
	rclcpp_action::CancelResponse handle_cancel_sequence( const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle){
		RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	void handle_accepted_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle){
		std::cout << "In handle_accepted_sequence \n";
		std::thread{std::bind(&OffboardController::execute_sequence, this, _1), goal_handle}.detach();
	}
	
	std::vector<Vector3D> get_scanWaypoints(double scan_height, double sensor_width, std::vector<geometry_msgs::msg::Pose2D> polygon_vertices);
	std::vector<ExecuteAtom> generateFlyAtomFromVector(std::vector<Vector3D> &gps_positions);
	/*
	*ACTIONS START
	*/
	//The Sequence execution which is called for every action
	void execute_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle);

	int execute_flyHomeAndLand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_flyDistance2Ground(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_turn2angle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_turn(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_hoverRight(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_hoverLeft(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_CirclePoI(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_searchArea(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_startDetection(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);
	
	int execute_stopDetection(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_takeImage(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_waypoint3D_step(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_waypoint2D(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_ascend(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_descend(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_flyAboveHeighestPoint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_waypoint3D(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);
	
	int execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg,  bool final_action = false);
	
	int execute_landing(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_hover(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_PanoramaPicture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

	int execute_scanArea(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg, bool final_action = false);

/*
* ACTIONS END
*/
	void publish_position_feedback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle);
	void get_origin(const std::shared_ptr<auspex_msgs::srv::GetOrigin::Request> request, std::shared_ptr<auspex_msgs::srv::GetOrigin::Response> response);
	/**
	 * @brief callback for set_origin service --> actually just setting home. NED origin should stay the same
	 */
	void set_origin(const std::shared_ptr<auspex_msgs::srv::SetOrigin::Request> request, std::shared_ptr<auspex_msgs::srv::SetOrigin::Response> response);

	void handle_user_command(const UserCommand::SharedPtr msg);

	void handle_add_action(const std::shared_ptr<auspex_msgs::srv::AddAction::Request> request, std::shared_ptr<auspex_msgs::srv::AddAction::Response> response);
};//Class end

#endif 