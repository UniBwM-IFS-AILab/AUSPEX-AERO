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
#include "auspex_msgs/msg/platform_command.hpp"

#include "auspex_msgs/srv/get_origin.hpp"
#include "auspex_msgs/srv/set_origin.hpp"
#include "auspex_msgs/srv/get_altitude.hpp"
#include "auspex_msgs/srv/add_action.hpp"

#include "auspex_msgs/srv/insert_knowledge.hpp"
#include "auspex_msgs/srv/exists_knowledge.hpp"

#include "msg_context_cpp/message_loader.hpp"

// Base Class
#include "auspex_fci/fc_interface_base.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include "auspex_fci/status_listener_base.hpp"

// For MavSDk
#include "auspex_fci/mavsdk/fc_interface_mavsdk.hpp"
#include "auspex_fci/mavsdk/position_listener_mavsdk.hpp"
#include "auspex_fci/mavsdk/status_listener_mavsdk.hpp"

// For ANAFI
#include "auspex_fci/anafi/fc_interface_anafi.hpp"
#include "auspex_fci/anafi/position_listener_anafi.hpp"
#include "auspex_fci/anafi/status_listener_anafi.hpp"



// following include is for transforming gps to NED coordinates
#include "auspex_fci/geodetic_converter.hpp"

// Nodes for listening to FC messages
#include "auspex_obc/drone_state_publisher.hpp"

#ifdef SWITCH_CAM_HEADER
	#if SWITCH_CAM_HEADER == 1
		#include <auspex_ocs/cam_publisher_simUE.hpp>
		typedef SimCamPublisherUE CamPublisher;
	#elif SWITCH_CAM_HEADER == 2
		#include <auspex_ocs/cam_publisher_rpi5.hpp>
		typedef RPI5CamPublisher CamPublisher;
	#elif SWITCH_CAM_HEADER == 3
		#include <auspex_ocs/cam_publisher_rpi.hpp>
		typedef RPICamPublisher CamPublisher;
	#elif SWITCH_CAM_HEADER == 4
		#include <auspex_ocs/cam_publisher_mk_smart.hpp>
		typedef MkSmartCamPublisher CamPublisher;
	#elif SWITCH_CAM_HEADER == 5
		#include <auspex_ocs/cam_publisher_simIS.hpp>
		typedef SimCamPublisherIS CamPublisher;
	#elif SWITCH_CAM_HEADER == 0
		#include <auspex_ocs/cam_publisher_empty.hpp>
		typedef EmptyCamPublisher CamPublisher;
	#endif
#endif

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
using std::placeholders::_1;


// msg
using ObjectKnowledge = auspex_msgs::msg::ObjectKnowledge;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using ExecuteAtom = auspex_msgs::msg::ExecuteAtom;
using AltitudeLevel = auspex_msgs::msg::AltitudeLevel;
using PlatformCommand = auspex_msgs::msg::PlatformCommand;

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

	OffboardController(std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener,
					std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener,
					std::shared_ptr<FC_Interface_Base> fc_interface,
					std::shared_ptr<DroneStatePublisher> drone_state_publisher,
					std::shared_ptr<CamPublisher> cam_publisher,
					std::string name_prefix = "") ;

	/**
	 * @brief Destructor - properly cleanup OBC resources
	 */
	~OffboardController() {
		// Cancel timer to prevent further callbacks
		if (timer_) {
			timer_->cancel();
			timer_.reset();
		}

		// Clear the execution queue safely
		{
			std::lock_guard<std::mutex> lock(execute_queue_lock);
			executeSequenceQueue.clear();
		}

		// Reset service and action server pointers
		sequence_action_server_.reset();
		get_origin_service_.reset();
		set_origin_service_.reset();
		handle_add_action_service_.reset();
		get_altitude_client.reset();
		platform_command_listener_.reset();

		// Reset shared pointers to managed objects
		vehicle_status_listener_.reset();
		position_listener_.reset();
		drone_state_publisher_.reset();
		camera_publisher_.reset();
		fc_interface_.reset();
		gps_converter_.reset();

		RCLCPP_INFO(get_logger(), "OffboardController destructor completed");
	}

	double getHeightAMSL(AltitudeLevel level, double value);
	double get_groundHeight_amsl();
	void publish_heartbeat();
	bool checkForExternalCommand();
	void start_action_server();
	void start_services();


private:
	enum RETURN_VALUE { action_critical_failure = -2, action_failure = -1, action_completed = 0, goal_canceled = 5, action_paused = 6, action_splitted =7, action_unkown = 8 };

	int current_action_index = 0;
	std::mutex execute_queue_lock;

	// pointer to action server objects
	rclcpp_action::Server<ExecuteSequence>::SharedPtr sequence_action_server_;

	// pointer to services
	rclcpp::Service<auspex_msgs::srv::GetOrigin>::SharedPtr get_origin_service_;
	rclcpp::Service<auspex_msgs::srv::SetOrigin>::SharedPtr set_origin_service_;
	rclcpp::Service<auspex_msgs::srv::AddAction>::SharedPtr handle_add_action_service_;

	rclcpp::Client<GetAltitude>::SharedPtr get_altitude_client;

	rclcpp::Subscription<PlatformCommand>::SharedPtr platform_command_listener_;

	rclcpp::TimerBase::SharedPtr timer_;

	std::shared_ptr<GeodeticConverter> gps_converter_;

	bool is_flying_ = false;
	bool is_height_data_available = false;
	std::string name_prefix_;

	/*
	*ACTIONS START
	*/
	//The Sequence execution which is called for every action
	void execute_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle);

	int execute_flyHomeAndLand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_flyDistance2Ground(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_turn2angle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_turn_by_angle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_hoverRight(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_hoverLeft(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_CirclePoI(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_searchArea(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_startCapture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_stopCapture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_takeImage(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_waypoint3D_step(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_waypoint2D(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_ascend(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_descend(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_flyAboveHeighestPoint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_waypoint3D(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_landing(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_hover(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_PanoramaPicture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

	int execute_scanArea(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg);

/*
* ACTIONS END
*/
	std::vector<Vector3D> get_scanWaypoints(double scan_height, double sensor_width, std::vector<geometry_msgs::msg::Pose2D> polygon_vertices);

	std::vector<ExecuteAtom> generateFlyAtomFromVector(std::vector<Vector3D> &gps_positions);

	void publish_position_feedback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle);

	void get_origin(const std::shared_ptr<auspex_msgs::srv::GetOrigin::Request> request, std::shared_ptr<auspex_msgs::srv::GetOrigin::Response> response);

	void set_origin(const std::shared_ptr<auspex_msgs::srv::SetOrigin::Request> request, std::shared_ptr<auspex_msgs::srv::SetOrigin::Response> response);

	void handle_platform_command(const PlatformCommand::SharedPtr msg);

	void handle_add_action(const std::shared_ptr<auspex_msgs::srv::AddAction::Request> request, std::shared_ptr<auspex_msgs::srv::AddAction::Response> response);

	// Action Server Callbacks
	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const typename ExecuteSequence::Goal> goal){
		RCLCPP_INFO(this->get_logger(), "Received new Goal Request");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse handle_cancel_sequence( const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle){
		RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle){
		RCLCPP_INFO(this->get_logger(), "Accepted Sequence Goal");
		std::thread{std::bind(&OffboardController::execute_sequence, this, _1), goal_handle}.detach();
	}
};//Class end

#endif