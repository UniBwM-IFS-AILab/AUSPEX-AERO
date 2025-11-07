//Load header files
#include"auspex_obc/obc_node.h"

#include <sstream> // For std::istringstream
#include <cmath>   // For mathematical functions
#include <limits>  // For std::numeric_limits


OffboardController::OffboardController(	std::shared_ptr<VehicleStatusListener_Base> status_listener,
										std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener,
										std::shared_ptr<FC_Interface_Base> fc_interface,
										std::shared_ptr<PlatformStatePublisher> state_publisher,
										std::shared_ptr<CameraController> cam_controller,
										std::shared_ptr<ExternalDataListener> extdata_listener,
										std::string platform_id,
										std::string payload) : Node(platform_id+ "_" + "auspex_controller") {

	platform_id_ = platform_id;
	status_listener_ = status_listener;
	position_listener_ = position_listener;
	camera_controller_ = cam_controller;
	state_publisher_ = state_publisher;
	fc_interface_ = fc_interface;
	extdata_listener_ = extdata_listener;
	payload_ = payload;

	this->position_listener_->set_recent_platform_state("INITIALIZING");

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	cmd_listener_ =  this->create_subscription<PlatformCommand>(platform_id + "/platform_command", qos, std::bind(&OffboardController::handle_platform_command, this, _1));
	
	gps_converter_ = std::make_shared<GeodeticConverter>(position_listener_->get_recent_home_msg()->latitude_deg, position_listener_->get_recent_home_msg()->longitude_deg, position_listener_->get_recent_home_msg()->absolute_altitude_m);
	fc_interface_->set_gps_converter(gps_converter_);
	fc_interface_->set_external_data_listener(extdata_listener_);
	
	RCLCPP_INFO(this->get_logger(), "Set GPS Origin to configured FC origin (%lf, %lf, %lf)", position_listener_->get_recent_home_msg()->latitude_deg, position_listener_->get_recent_home_msg()->longitude_deg, position_listener_->get_recent_home_msg()->absolute_altitude_m);
	
	timer_ = this->create_wall_timer(250ms, std::bind(&OffboardController::publish_heartbeat, this));
	
	this->start_action_server();
	this->start_services();
	
	this->get_altitude_client = this->create_client<GetAltitude>("/auspex_get_altitude");

	this->state_publisher_->start_publish(this->position_listener_, this->status_listener_);
	this->camera_controller_->start_capture();
	this->position_listener_->set_recent_platform_state("LANDED");
}

void OffboardController::publish_heartbeat(){
	fc_interface_->publish_offboard_heartbeat();
}

void OffboardController::start_action_server(){
	using namespace std::placeholders;
	RCLCPP_INFO(this->get_logger(), "Starting action server:");
	sequence_action_server_ = rclcpp_action::create_server<ExecuteSequence>(
		this,
		platform_id_ + "/action_sequence",
		std::bind(&OffboardController::handle_goal, this, _1, _2),
		std::bind(&OffboardController::handle_cancel_sequence, this, _1),
		std::bind(&OffboardController::handle_accepted_sequence, this, _1));
		RCLCPP_INFO(get_logger(), "Action servers are ready. \n");
}

void OffboardController::start_services(){
	using namespace std::placeholders;

	RCLCPP_INFO(this->get_logger(), "Starting services...");
	get_origin_service_ = this -> create_service<auspex_msgs::srv::GetOrigin>(platform_id_ + "/srv/get_origin", std::bind(&OffboardController::get_origin, this, _1, _2));
	set_origin_service_ = this -> create_service<auspex_msgs::srv::SetOrigin>(platform_id_ + "/srv/set_origin", std::bind(&OffboardController::set_origin, this, _1, _2));
	handle_add_action_service_ = this -> create_service<auspex_msgs::srv::AddAction>(platform_id_ + "/srv/add_action", std::bind(&OffboardController::handle_add_action, this, _1, _2));
	RCLCPP_INFO(this->get_logger(), "Services are ready.");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////	Action Methods BEGIN	/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OffboardController::execute_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle){
	RCLCPP_INFO(this->get_logger(),"In Execute Sequence" );

	rclcpp::Rate loop_rate(10);
	rclcpp::Rate loop_rate_slow(1);
	int return_code = OffboardController::RETURN_VALUE::action_critical_failure;
	
	if(status_listener_->get_paused_from_extern()){
		if(current_goal_handle_ != nullptr){
			RCLCPP_INFO(this->get_logger(), "Aborting previous goal due to new incoming goal.");
			execute_queue_lock.lock();
			goal_overwrite_ = true;
			executeSequenceQueue.clear();
			execute_queue_lock.unlock();
			loop_rate_slow.sleep();
			loop_rate_slow.sleep();
		}
	}

	current_goal_handle_ = goal_handle;

	auto executeAtomsVector = current_goal_handle_->get_goal()->execute_atoms;
	auto result = std::make_shared<ExecuteSequence::Result>();

	execute_queue_lock.lock();
	executeSequenceQueue.clear();
	execute_queue_lock.unlock();
	
	for(auto &atom: executeAtomsVector){
		executeSequenceQueue.emplace_back(atom);
	}
	
	if(executeSequenceQueue.empty()){
		RCLCPP_ERROR(this->get_logger(),"ERROR: Received empty execute atoms list.");
		return;
	}
	
	current_action_index = 0;
	int initial_action_count = executeAtomsVector.size();
	
	ExecuteAtom atom;
	
	while(!executeSequenceQueue.empty()){
		if (current_goal_handle_->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			current_goal_handle_->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Action Sequence canceled");
			return;
		}
		
		if(status_listener_->get_paused_from_extern()){
			loop_rate.sleep();
			publish_position_feedback(current_goal_handle_);
			continue;
		}
		
		execute_queue_lock.lock();
		atom = executeSequenceQueue.front();
		execute_queue_lock.unlock();
		
		if(!is_flying_ && atom.action_type != "take_off" && atom.action_type != "start_detection" && atom.action_type != "stop_detection"){
			RCLCPP_INFO(this->get_logger(), "Can not execute action %s because the drone is not flying.", atom.action_type.c_str());
			return;
		}
		
		RCLCPP_INFO(this->get_logger(), "Current action in sequence[%d]: %s",current_action_index, atom.action_type.c_str());
		
		if(atom.action_type == "take_off")
			return_code = execute_takeoff(current_goal_handle_, &atom);
		else if(atom.action_type == "land")
			return_code = execute_landing(current_goal_handle_, &atom);
		else if(atom.action_type == "fly_3D")
			return_code = execute_waypoint3D(current_goal_handle_, &atom);
		else if(atom.action_type == "hover")
			return_code = execute_hover(current_goal_handle_, &atom);
		else if(atom.action_type == "scan_area_uav")
			return_code = execute_scanArea(current_goal_handle_, &atom);
		else if(atom.action_type == "ascend")
			return_code = execute_ascend(current_goal_handle_, &atom);
		else if(atom.action_type == "descend")
			return_code = execute_descend(current_goal_handle_, &atom);
		else if(atom.action_type == "fly_above_highest_point")
			return_code = execute_flyAboveHeighestPoint(current_goal_handle_, &atom);
		else if(atom.action_type == "fly_2D")
			return_code = execute_waypoint2D(current_goal_handle_, &atom);
		else if(atom.action_type == "fly_step_3D")
			return_code = execute_waypoint3D_step(current_goal_handle_, &atom);
		else if(atom.action_type == "search_area_uav")
			return_code = execute_searchArea(current_goal_handle_, &atom);
		else if(atom.action_type == "start_detection")
			return_code = execute_startCapture(current_goal_handle_, &atom);
		else if(atom.action_type == "stop_detection")
			return_code = execute_stopCapture(current_goal_handle_, &atom);
		else if(atom.action_type == "circle_poi")
			return_code = execute_CirclePoI(current_goal_handle_, &atom);
		else if(atom.action_type == "hover_right")
			return_code = execute_hoverRight(current_goal_handle_, &atom);
		else if(atom.action_type == "hover_left")
			return_code = execute_hoverLeft(current_goal_handle_, &atom);
		else if(atom.action_type == "turn")
			return_code = execute_turn_by_angle(current_goal_handle_, &atom);
		else if(atom.action_type == "fly_at_ground_distance")
			return_code = execute_flyDistance2Ground(current_goal_handle_, &atom);
		else if(atom.action_type == "return_home_and_land")
			return_code = execute_flyHomeAndLand(current_goal_handle_, &atom);
		else if(atom.action_type == "release_object")
			return_code = execute_releaseObject(current_goal_handle_, &atom);
		else{
			RCLCPP_INFO(this->get_logger(), "Unknown action type: %s", atom.action_type.c_str());
			return_code = OffboardController::RETURN_VALUE::action_unknown;
		}
		
		//Locks the queue checks if it was the last action in it and removes it from the queue.
		execute_queue_lock.lock();
		bool final_action = false;
		if(return_code != OffboardController::RETURN_VALUE::action_paused && return_code != OffboardController::RETURN_VALUE::action_splitted && return_code != OffboardController::RETURN_VALUE::action_unknown){
			final_action = executeSequenceQueue.size() == 1;
			if(!executeSequenceQueue.empty()){
				executeSequenceQueue.pop_front();
			}
		}
		execute_queue_lock.unlock();
		
		switch (return_code) {
			case OffboardController::RETURN_VALUE::action_completed:

				current_action_index++;
				RCLCPP_INFO(this->get_logger(), "Action %d out of %ld in sequence successfully completed", current_action_index, initial_action_count);
				if(final_action){
					RCLCPP_INFO(this->get_logger(), "Sequence successfully completed");
					current_goal_handle_->succeed(result);
					return;
				}
				break;
			
			case OffboardController::RETURN_VALUE::goal_canceled:

				RCLCPP_INFO(this->get_logger(), "Sequence canceled during action %d out of %ld",(current_action_index+1), initial_action_count);
				RCLCPP_INFO(this->get_logger(), "Successfully completed %d actions of the sequence",(current_action_index+1));
				this->publish_position_feedback(current_goal_handle_);
				current_goal_handle_->canceled(result);
				return ;

			case OffboardController::RETURN_VALUE::action_paused:{

				RCLCPP_INFO(this->get_logger(), "Action Paused from extern. Waiting for resume...");
				break;

			}
			case OffboardController::RETURN_VALUE::action_critical_failure:{

				RCLCPP_INFO(this->get_logger(), "Recent Action failed critically. Engaging failsafe mode");
				if(is_flying_){
					fc_interface_->publish_position_control_mode();
				}
				this->publish_position_feedback(current_goal_handle_);
				// Try to cancel, but catch exception if goal is already aborted
				try {
					current_goal_handle_->abort(result);
				} catch (const rclcpp::exceptions::RCLError& e) {
					RCLCPP_INFO(this->get_logger(), "Goal already in terminal state: %s", e.what());
				}
				return ;

			}
			case OffboardController::RETURN_VALUE::action_unknown:{

				RCLCPP_INFO(this->get_logger(), "Unknown Action type in sequence. Returning...");
				if(is_flying_){
					fc_interface_->publish_position_control_mode();
				}
				this->publish_position_feedback(current_goal_handle_);
				// Try to cancel, but catch exception if goal is already aborted
				try {
					current_goal_handle_->abort(result);
				} catch (const rclcpp::exceptions::RCLError& e) {
					RCLCPP_INFO(this->get_logger(), "Goal already in terminal state: %s", e.what());
				}
				return ;

			}
			case OffboardController::RETURN_VALUE::action_failure:{

				if(is_flying_){
					fc_interface_->publish_position_control_mode();
				}
				RCLCPP_INFO(this->get_logger(), "Recent Action failed. Continuing with next action...");
				break;

			}
		}
	}//END-WHILE

	this->publish_position_feedback(current_goal_handle_);
	if(goal_overwrite_){
		result->error_code = 5; //overwrite
		current_goal_handle_->succeed(result);
		position_listener_->set_recent_platform_state("AIRBORNE");
		status_listener_->set_paused_from_extern(false);
		goal_overwrite_ = false;
	}
				
	execute_queue_lock.unlock();
	return ;
}

int OffboardController::execute_flyHomeAndLand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_flyDistanceGround \n");

	int result_atom = this->execute_waypoint3D_step(goal_handle, execute_msg);//amsl

	if(result_atom == OffboardController::RETURN_VALUE::action_completed){
		execute_msg->goal_pose.position.altitude = position_listener_->get_recent_home_msg()->absolute_altitude_m + 4;
		execute_msg->altitude_level.value = AltitudeLevel::AMSL;
		result_atom = this->execute_descend(goal_handle, execute_msg);

		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			result_atom = this->execute_landing(goal_handle, execute_msg);
		}
	}
	return result_atom;
}

int OffboardController::execute_flyDistance2Ground(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_flyDistanceGround \n");

	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	geographic_msgs::msg::GeoPose pose;
	pose = execute_msg->goal_pose;
	// TODO
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_hoverLeft(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In hover_left \n");

	double north, east, down;
	gps_converter_->geodetic2Ned(execute_msg->goal_pose.position.latitude, execute_msg->goal_pose.position.longitude, position_listener_->get_recent_gps_msg()->absolute_altitude_m, &north, &east, &down); // given alt above mean sea level

	double current_north, current_east, current_down;
	// receive current NED coordinates from FC
	current_north = position_listener_->get_recent_ned_msg()->position_body.x_m;
	current_east = position_listener_->get_recent_ned_msg()->position_body.y_m;
	current_down = position_listener_->get_recent_ned_msg()->position_body.z_m;

	north = gps_converter_->discretize(north,5.0);
	east = gps_converter_->discretize(east,5.0);
	down = gps_converter_->discretize(down,5.0);

	double roll, pitch,goal_yaw;

	goal_yaw = atan2(east -current_east,north - current_north) + M_PI/2.0; // [-PI:PI]
	goal_yaw = gps_converter_->discretize(goal_yaw,5.0);

	tf2::Quaternion q;
	q.setRPY(roll,pitch,goal_yaw);

	execute_msg->goal_pose.orientation.x = q.x();
	execute_msg->goal_pose.orientation.y = q.y();
	execute_msg->goal_pose.orientation.z = q.z();
	execute_msg->goal_pose.orientation.w = q.w();

	int result_atom = this->execute_turn2angle(goal_handle, execute_msg);

	if(result_atom != OffboardController::RETURN_VALUE::action_completed){
		return result_atom;
	}

	//Now hover to position
	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();

	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	double goal_height = position_listener_->get_recent_gps_msg()->absolute_altitude_m;

	double distance2D = 0.0;
	double max_distance = 30.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = execute_msg->goal_pose;
		double current_distance = gps_converter_->geodeticDistance2D(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

		if (current_distance > max_distance) {
			double fraction = max_distance / current_distance;
			intermediate_goal.position.latitude = position_listener_->get_recent_gps_msg()->latitude_deg + fraction * (intermediate_goal.position.latitude -  position_listener_->get_recent_gps_msg()->latitude_deg);
    		intermediate_goal.position.longitude = position_listener_->get_recent_gps_msg()->longitude_deg + fraction * (intermediate_goal.position.longitude - position_listener_->get_recent_gps_msg()->longitude_deg);
		}

		distance2D = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, goal_height, HEADING::UNCHANGED, execute_msg->speed_m_s);//alt amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance2D <= 0.5) {
			if (std::fabs(goal_height - position_listener_->get_recent_gps_msg()->absolute_altitude_m) < 2.0){
				loop_rate.sleep();
				break;
			}
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_hoverRight(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In hover_right \n");

	double north, east, down;
	gps_converter_->geodetic2Ned(execute_msg->goal_pose.position.latitude, execute_msg->goal_pose.position.longitude, position_listener_->get_recent_gps_msg()->absolute_altitude_m, &north, &east, &down);

	double current_north, current_east, current_down;
	// receive current NED coordinates from FC
	current_north = position_listener_->get_recent_ned_msg()->position_body.x_m;
	current_east = position_listener_->get_recent_ned_msg()->position_body.y_m;
	current_down = position_listener_->get_recent_ned_msg()->position_body.z_m;

	north = gps_converter_->discretize(north,5.0);
	east = gps_converter_->discretize(east,5.0);
	down = gps_converter_->discretize(down,5.0);

	double roll, pitch,goal_yaw;

	goal_yaw = atan2(east -current_east,north - current_north) - M_PI/2.0; // [-PI:PI]
	goal_yaw = gps_converter_->discretize(goal_yaw,5.0);

	tf2::Quaternion q;
	q.setRPY(roll,pitch,goal_yaw);

	execute_msg->goal_pose.orientation.x = q.x();
	execute_msg->goal_pose.orientation.y = q.y();
	execute_msg->goal_pose.orientation.z = q.z();
	execute_msg->goal_pose.orientation.w = q.w();

	int result_atom = this->execute_turn2angle(goal_handle, execute_msg);

	if(result_atom != OffboardController::RETURN_VALUE::action_completed){
		return result_atom;
	}

	//Now hover to position
	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();

	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	double distance2D = 0.0;
	double max_distance = 30.0;
	double goal_height = position_listener_->get_recent_gps_msg()->absolute_altitude_m;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = execute_msg->goal_pose;
		double current_distance = gps_converter_->geodeticDistance2D(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

		if (current_distance > max_distance) {
			double fraction = max_distance / current_distance;
			intermediate_goal.position.latitude = position_listener_->get_recent_gps_msg()->latitude_deg + fraction * (intermediate_goal.position.latitude -  position_listener_->get_recent_gps_msg()->latitude_deg);
    		intermediate_goal.position.longitude = position_listener_->get_recent_gps_msg()->longitude_deg + fraction * (intermediate_goal.position.longitude - position_listener_->get_recent_gps_msg()->longitude_deg);
		}

		distance2D = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, goal_height, HEADING::UNCHANGED, execute_msg->speed_m_s);//alt above mean sea level

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance2D <= 0.5) {
			if (std::fabs(goal_height - position_listener_->get_recent_gps_msg()->absolute_altitude_m) < 2.0){
				loop_rate.sleep();
				break;
			}
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_turn2WP(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_turn2WP \n");

	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	geographic_msgs::msg::GeoPose pose = execute_msg->goal_pose;
	auto result = std::make_shared<ExecuteSequence::Result>();
	
	rclcpp::Rate loop_rate(5);
	// Set max turn rate based on FC type - ARDUPILOT uses 4.0 rad/s, others use 1.0 rad/s
	double max_turn_rate_rad_s = 1.0;
	if (fc_interface_->get_FC_TYPE().find("ARDUPILOT") != std::string::npos) {
		max_turn_rate_rad_s = 4.0;
	}
	double dt = duration_cast<duration<double>>(loop_rate.period()).count();  // Time step

	// Calculate target yaw angle towards the waypoint
	double north, east, down;
	gps_converter_->geodetic2Ned(pose.position.latitude, pose.position.longitude, 
								position_listener_->get_recent_gps_msg()->absolute_altitude_m, 
								&north, &east, &down);

	// Get current position
	double current_north = position_listener_->get_recent_ned_msg()->position_body.x_m;
	double current_east = position_listener_->get_recent_ned_msg()->position_body.y_m;
	
	// Calculate target yaw towards waypoint
	double target_yaw = atan2(east - current_east, north - current_north);
	
	// Get current yaw
	tf2::Quaternion current_q(position_listener_->get_recent_ned_msg()->q.x,
							 position_listener_->get_recent_ned_msg()->q.y,
							 position_listener_->get_recent_ned_msg()->q.z,
							 position_listener_->get_recent_ned_msg()->q.w);
	tf2::Matrix3x3 m(current_q);
	double current_roll, current_pitch, current_yaw;
	m.getRPY(current_roll, current_pitch, current_yaw);

	RCLCPP_INFO(this->get_logger(), "Turning from current yaw: %.3f to target yaw: %.3f", 
				current_yaw * 180.0 / M_PI, target_yaw * 180.0 / M_PI);

	while (rclcpp::ok()) {
		loop_rate.sleep();
		
		// Get current yaw again for this iteration
		tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q.x,
						 position_listener_->get_recent_ned_msg()->q.y,
						 position_listener_->get_recent_ned_msg()->q.z,
						 position_listener_->get_recent_ned_msg()->q.w);
		tf2::Matrix3x3 matrix(q);
		double roll, pitch, yaw;
		matrix.getRPY(roll, pitch, yaw);
		
		// Calculate shortest angular distance to target
		double yaw_deviation = target_yaw - yaw;
		// Normalize to [-PI, PI]
		while (yaw_deviation > M_PI) yaw_deviation -= 2.0 * M_PI;
		while (yaw_deviation < -M_PI) yaw_deviation += 2.0 * M_PI;
		
		// Calculate the yaw step for this iteration (limited by max turn rate)
		double max_step = max_turn_rate_rad_s * dt;
		double yaw_step = std::min(std::abs(yaw_deviation), max_step);
		if (yaw_deviation < 0) yaw_step = -yaw_step;
		
		// Calculate intermediate yaw target for this iteration
		double intermediate_yaw = yaw + yaw_step;
		
		// Send setpoint to FC interface
		double remaining_deviation = fc_interface_->move_to_angle(roll, pitch, intermediate_yaw, max_turn_rate_rad_s);
		
		// Check for cancellation
		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_DEBUG(this->get_logger(), "Turn to waypoint canceled");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		// Check for external command pause
		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		// Check if we've reached the target yaw (within 0.05 radians ≈ 3 degrees)
		if (std::abs(yaw_deviation) <= 0.05) {
			RCLCPP_INFO(this->get_logger(), "Turn to waypoint completed. Final deviation: %.3f degrees", std::abs(yaw_deviation) * 180.0 / M_PI);
			break;
		}
	}
	
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_turn_by_angle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_turn \n");

	tf2::Quaternion q_goal(	execute_msg->goal_pose.orientation.x,
							execute_msg->goal_pose.orientation.y,
							execute_msg->goal_pose.orientation.z,
							execute_msg->goal_pose.orientation.w);

	double roll, pitch, yaw, goal_yaw;
	tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q.x,position_listener_->get_recent_ned_msg()->q.y,position_listener_->get_recent_ned_msg()->q.z,position_listener_->get_recent_ned_msg()->q.w);
	q = q * q_goal; //add to current orientation

	execute_msg->goal_pose.orientation.x = q.x();
	execute_msg->goal_pose.orientation.y = q.y();
	execute_msg->goal_pose.orientation.z = q.z();
	execute_msg->goal_pose.orientation.w = q.w();

	return this->execute_turn2angle(goal_handle, execute_msg);
}

int OffboardController::execute_turn2angle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_turn2angle \n");

	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	double x = position_listener_->get_recent_ned_msg()->position_body.x_m;
	double y = position_listener_->get_recent_ned_msg()->position_body.y_m;
	double z = position_listener_->get_recent_ned_msg()->position_body.z_m;
	double roll, pitch , yaw, goal_yaw;

	tf2::Quaternion q(execute_msg->goal_pose.orientation.x,execute_msg->goal_pose.orientation.y, execute_msg->goal_pose.orientation.z, execute_msg->goal_pose.orientation.w);

	tf2::Matrix3x3 m(q);
	m.getRPY(roll, pitch, goal_yaw);

	if(is_flying_){
		fc_interface_->move_to_ned(x, y, z,0.0,0.0,goal_yaw);
	}else{
		return OffboardController::RETURN_VALUE::action_critical_failure;
	}

	int tick_count= 0;
	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();

	while (rclcpp::ok()){
		loop_rate.sleep();

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		tf2::Quaternion q(position_listener_->get_recent_ned_msg()->q.x, position_listener_->get_recent_ned_msg()->q.y, position_listener_->get_recent_ned_msg()->q.z, position_listener_->get_recent_ned_msg()->q.w);
		tf2::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);

		if(std::fabs(yaw - goal_yaw) < 0.0035){
			tick_count++;
		}

		if (rclcpp::ok()  && tick_count > 4) {
			loop_rate.sleep();
			RCLCPP_INFO(this->get_logger(), "Turn in position succeeded.");
			break;
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

/**
 * @brief only works with FC at the moment.
 */
int OffboardController::execute_CirclePoI(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_CirclePoI \n");

	if(checkForExternalCommand()){
		fc_interface_->publish_position_control_mode();
		return OffboardController::RETURN_VALUE::action_paused;
	}

	double goal_height = execute_msg->goal_pose.position.altitude;
	switch (execute_msg->altitude_level.value) {
		case AltitudeLevel::AMSL:
			goal_height -= position_listener_->get_recent_home_msg()->absolute_altitude_m;
			break;
		case AltitudeLevel::AGL:
			goal_height = (get_groundHeight_amsl() + goal_height) - position_listener_->get_recent_home_msg()->absolute_altitude_m;
			break;
		case AltitudeLevel::REL:
			goal_height = position_listener_->get_recent_ned_msg()->position_body.z_m + goal_height;
			break;
		default:
			RCLCPP_ERROR(this->get_logger(), "Unknown altitude level: %d", execute_msg->altitude_level.value);
			return OffboardController::RETURN_VALUE::action_critical_failure;
	}

	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();

	double distance = 0.0;
	int current_times = 0;

	//uint16 VEHICLE_CMD_DO_ORBIT = 34	Radius [m] |Velocity [m/s] |Yaw behaviour |Empty |latitude/X |longitude/Y |Altitude/Z |
	fc_interface_->publish_circle_poi(execute_msg->radius, execute_msg->speed_m_s, position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, goal_height);

	auto start_time = this->get_clock()->now();
	auto current_time = this->get_clock()->now();

	while (rclcpp::ok()) {
		loop_rate.sleep();

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_INFO(this->get_logger(), "Canceled Circle POI");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		current_time = this->get_clock()->now();
		if (rclcpp::ok() && ((current_time - start_time).nanoseconds()/1000000.0 > (execute_msg->hover_duration))) {
			loop_rate.sleep();
			break;
		}
	}

	fc_interface_->publish_position_control_mode();
	fc_interface_->publish_offboard_control_mode();
	fc_interface_->publish_position_control_mode();
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_flyAboveHeighestPoint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_flyAboveHeighestPoint \n");

	double current_alt = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
	int result_atom = 0;
	if(current_alt > execute_msg->goal_pose.position.altitude){
		result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			result_atom = this->execute_descend(goal_handle, execute_msg);
		}
	}else if (current_alt < execute_msg->goal_pose.position.altitude){
		result_atom = this->execute_ascend(goal_handle, execute_msg);
		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
		}
	}else{
		result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
	}
	return result_atom;
}

int OffboardController::execute_scanArea(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(), "In execute_scanArea");

	auto result = std::make_shared<ExecuteSequence::Result>();
	double scan_height = execute_msg->height - position_listener_->get_recent_home_msg()->absolute_altitude_m;
	execute_msg->altitude_level.value = AltitudeLevel::AMSL;
	std::vector<geometry_msgs::msg::Pose2D> polygon_vertices = execute_msg->scan_polygon_vertices;

	//the sensor FOV in degree// 30 degree means +/- 15
    constexpr double SENSOR_FOV_HOR = 30.0;

	//compute footprint
	double sensor_footprint = (tan(((SENSOR_FOV_HOR/2.0)*M_PI)/180.0) * scan_height) * 2.0;//in metres

	//abort if there are not enough vertices given
	if(polygon_vertices.size() < 3){
		RCLCPP_ERROR(this->get_logger(), "Error: Not enough vertices specified for scanning.");
		return OffboardController::RETURN_VALUE::action_failure;
	}
	std::vector<Vector3D> scanWaypoints = this->get_scanWaypoints(execute_msg->height, sensor_footprint, polygon_vertices, ScanPattern::LAWNMOWER);
	std::vector<ExecuteAtom> executeAtoms_wp = generateFlyAtomFromVector(scanWaypoints);

	for(auto &wp: executeAtoms_wp){
		int result_atom = execute_waypoint3D(goal_handle, &wp);
		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			continue;
		}else{
			return result_atom;
		}
	}
	RCLCPP_INFO(this->get_logger(), "Successfully scanned the specified area.");
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_searchArea(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg) {
    RCLCPP_INFO(this->get_logger(), "Executing search area...");

    auto result = std::make_shared<ExecuteSequence::Result>();
    double scan_height = execute_msg->height - position_listener_->get_recent_home_msg()->absolute_altitude_m;
    execute_msg->altitude_level.value = AltitudeLevel::AMSL;
    std::vector<geometry_msgs::msg::Pose2D> polygon_vertices = execute_msg->scan_polygon_vertices;

    // Sensor FOV (horizontal & vertical) in degrees
    constexpr double SENSOR_FOV_HOR = 30.0;

    // Compute sensor footprint in meters
    double sensor_footprint = 2.0 * tan((SENSOR_FOV_HOR / 2.0) * M_PI / 180.0) * scan_height;

    if (polygon_vertices.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Error: Not enough vertices specified for scanning.");
        return OffboardController::RETURN_VALUE::action_failure;
    }

    std::vector<Vector3D> scanWaypoints = this->get_scanWaypoints(execute_msg->height, sensor_footprint, polygon_vertices, ScanPattern::SPIRAL);
    std::vector<ExecuteAtom> executeAtoms_wp = this->generateFlyAtomFromVector(scanWaypoints);
    RCLCPP_INFO(this->get_logger(), "Searching for object at height: %f meters", scan_height);

    int finished_wp = 0;
    for (auto &wp : executeAtoms_wp) {
        int result_atom = this->execute_waypoint3D_step(goal_handle, &wp);

        if (result_atom == OffboardController::RETURN_VALUE::action_completed) {
            finished_wp++;
            continue;
        }
        else if (result_atom == OffboardController::RETURN_VALUE::action_paused) {
            // Handle pause case: Store remaining waypoints and modify execute atom
            if ((executeAtoms_wp.size() - finished_wp) <= 2) {
                // Ensure at least 3 waypoints remain for next iteration
                finished_wp = std::max(0, static_cast<int>(executeAtoms_wp.size()) - 3);
            }

            // Find bounding box for remaining waypoints
            double x_min = 91.0, x_max = -91.0;
            double y_min = 181.0, y_max = -181.0;

            int skip_index = 0;
            for (auto &waypoint : scanWaypoints) {
                if (skip_index++ <= finished_wp) continue;

                x_min = std::min(x_min, waypoint.getX());
                x_max = std::max(x_max, waypoint.getX());
                y_min = std::min(y_min, waypoint.getY());
                y_max = std::max(y_max, waypoint.getY());
            }

            std::vector<geometry_msgs::msg::Pose2D> poses;
			poses.emplace_back();
			poses.back().x = x_min;
			poses.back().y = y_min;

			poses.emplace_back();
			poses.back().x = x_min;
			poses.back().y = y_max;

			poses.emplace_back();
			poses.back().x = x_max;
			poses.back().y = y_max;

			poses.emplace_back();
			poses.back().x = x_max;
			poses.back().y = y_min;
            {
                std::lock_guard<std::mutex> lock(execute_queue_lock);
                if (!executeSequenceQueue.empty()) {
                    executeSequenceQueue.front().scan_polygon_vertices = poses;
                }
            }
            return result_atom;
        }
        else {
            return result_atom;
        }
    }

    return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_waypoint3D_step(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_waypoint 2 Steps \n");

	geographic_msgs::msg::GeoPose pose;
	pose = execute_msg->goal_pose;
	double current_alt = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
	double goal_alt = pose.position.altitude; //amsl
	int result_atom = 0;

	// First, turn to face the waypoint before any movement
	RCLCPP_INFO(this->get_logger(), "Step 1: Turning to face waypoint");
	result_atom = this->execute_turn2WP(goal_handle, execute_msg);
	if(result_atom != OffboardController::RETURN_VALUE::action_completed){
		return result_atom;  // Return early if turn failed/was cancelled/paused
	}

	// Now execute the waypoint movement based on altitude difference
	if(current_alt > goal_alt){
		RCLCPP_INFO(this->get_logger(), "Step 2: Moving horizontally first, then descending");
		result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			result_atom = this->execute_descend(goal_handle, execute_msg);
		}
	}else if (current_alt < goal_alt){
		RCLCPP_INFO(this->get_logger(), "Step 2: Ascending first, then moving horizontally");
		result_atom = this->execute_ascend(goal_handle, execute_msg);
		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
		}
	}else{
		RCLCPP_INFO(this->get_logger(), "Step 2: Moving horizontally (same altitude)");
		result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
	}
	return result_atom;
}

//To execute a 2D motion on the same height.
int OffboardController::execute_ascend(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_ascend\n");
	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	//calculate the goal height in amsl
	double goal_height = getHeightAMSL(execute_msg->altitude_level, execute_msg->goal_pose.position.altitude);

	RCLCPP_INFO(this->get_logger(), "Ascending from %f to %f ", position_listener_->get_recent_gps_msg()->absolute_altitude_m, goal_height);

	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();
	double distance2D = 0.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		distance2D = fc_interface_->move_to_gps(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, goal_height, HEADING::UNCHANGED, execute_msg->speed_m_s); // goal_height in amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance2D <= 0.5) {
			if (std::fabs(goal_height - position_listener_->get_recent_gps_msg()->absolute_altitude_m) < 2.0){
				loop_rate.sleep();
				break;
			}
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_descend(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_descend\n");
	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	double goal_height = getHeightAMSL(execute_msg->altitude_level, execute_msg->goal_pose.position.altitude);

	RCLCPP_INFO(this->get_logger(), "Descending from %f to %f.", position_listener_->get_recent_gps_msg()->absolute_altitude_m, goal_height);

	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();
	double distance2D = 0.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		distance2D = fc_interface_->move_to_gps(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, goal_height, HEADING::UNCHANGED, execute_msg->speed_m_s);//alt should be amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance2D <= 0.5) {
			if (std::fabs(goal_height - position_listener_->get_recent_gps_msg()->absolute_altitude_m) < 2.0){
				loop_rate.sleep();
				break;
			}
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_waypoint2D(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_waypoint 2D \n");

	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	geographic_msgs::msg::GeoPose pose;
	pose = execute_msg->goal_pose;
	pose.position.altitude = position_listener_->get_recent_gps_msg()->absolute_altitude_m;

	RCLCPP_INFO(this->get_logger(), "Starting navigation to %lf, %lf, %lf", pose.position.latitude, pose.position.longitude, pose.position.altitude);

	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();
	double distance2D = 0.0;
	double max_distance = 15.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = pose;
		double current_distance = gps_converter_->geodeticDistance2D(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

		if (current_distance > max_distance) {
			double fraction = max_distance / current_distance;
			intermediate_goal.position.latitude = position_listener_->get_recent_gps_msg()->latitude_deg + fraction * (intermediate_goal.position.latitude -  position_listener_->get_recent_gps_msg()->latitude_deg);
    		intermediate_goal.position.longitude = position_listener_->get_recent_gps_msg()->longitude_deg + fraction * (intermediate_goal.position.longitude - position_listener_->get_recent_gps_msg()->longitude_deg);
		}

		HEADING heading = HEADING::UNCHANGED;
		if(current_distance > 2.5){
			heading = HEADING::WAYPOINT;
		}
		// NED frame doesnt reset once reaching a waypoint -> altitude is always above mean sea level
		distance2D = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, pose.position.altitude, heading, execute_msg->speed_m_s); //alt is in amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance2D <= 0.5) {
			if (std::fabs(pose.position.altitude - position_listener_->get_recent_gps_msg()->absolute_altitude_m) < 2.0){
				loop_rate.sleep();
				break;
			}
		}
	}

	if(execute_msg->goal_pose.orientation.x != 0.0 || execute_msg->goal_pose.orientation.y != 0.0 || execute_msg->goal_pose.orientation.z != 0.0 || execute_msg->goal_pose.orientation.w != 1.0){
		RCLCPP_INFO(this->get_logger(), "Executing turn to goal orientation");
		int result_atom = this->execute_turn2angle(goal_handle, execute_msg);
		if(result_atom != OffboardController::RETURN_VALUE::action_completed){
			return result_atom;
		}
	}

	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_waypoint3D(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_waypoint 3D\n");
	if(!fc_interface_->publish_offboard_control_mode()){
		return OffboardController::RETURN_VALUE::action_paused;
	}

	geographic_msgs::msg::GeoPose pose = execute_msg->goal_pose;

	RCLCPP_INFO(this->get_logger(), "Starting navigation to %lf, %lf, %lf", pose.position.latitude, pose.position.longitude, pose.position.altitude);
	double goal_height = getHeightAMSL(execute_msg->altitude_level, execute_msg->goal_pose.position.altitude);
	auto result = std::make_shared<ExecuteSequence::Result>();

	rclcpp::Rate loop_rate(5);
	double distance2D = 0.0;
	double max_distance = 15.0; //metres

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = pose;
		double current_distance = gps_converter_->geodeticDistance2D(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

		if (current_distance > max_distance) {
			double fraction = max_distance / current_distance;
			intermediate_goal.position.latitude = position_listener_->get_recent_gps_msg()->latitude_deg + fraction * (intermediate_goal.position.latitude -  position_listener_->get_recent_gps_msg()->latitude_deg);
    		intermediate_goal.position.longitude = position_listener_->get_recent_gps_msg()->longitude_deg + fraction * (intermediate_goal.position.longitude - position_listener_->get_recent_gps_msg()->longitude_deg);
		}

		HEADING heading = HEADING::UNCHANGED;
		if(current_distance > 3.0){
			heading = HEADING::WAYPOINT;
		}

		// NED frame doesnt reset once reaching a waypoint -> altitude is always above mean sea level
		distance2D = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, goal_height, heading, execute_msg->speed_m_s); //alt should be above mean sea level

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_DEBUG(this->get_logger(), "Navigation to current waypoint canceled");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance2D <= 0.5) {
			if (std::fabs(goal_height - position_listener_->get_recent_gps_msg()->absolute_altitude_m) < 2.0){
				loop_rate.sleep();
				break;
			}
		}

	}

	if(execute_msg->goal_pose.orientation.x != 0.0 || execute_msg->goal_pose.orientation.y != 0.0 || execute_msg->goal_pose.orientation.z != 0.0 || execute_msg->goal_pose.orientation.w != 1.0){
		RCLCPP_INFO(this->get_logger(), "Executing turn to goal orientation");
		int result_atom = this->execute_turn2angle(goal_handle, execute_msg);
		if(result_atom != OffboardController::RETURN_VALUE::action_completed){
			return result_atom;
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_hover(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_hover \n");

	auto result = std::make_shared<ExecuteSequence::Result>();

	rclcpp::Rate loop_rate(5);
	auto start_time = this->get_clock()->now();
	auto current_time = this->get_clock()->now();
	int current_times = 0;

	fc_interface_->publish_position_control_mode();

	while (rclcpp::ok()){
		loop_rate.sleep();
		current_times++;

		if(is_flying_){
			fc_interface_->publish_position_control_mode();
		}

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_INFO(this->get_logger(), "Hovering at current waypoint canceled");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		current_time = this->get_clock()->now();
		if (rclcpp::ok()  && ((current_time - start_time).nanoseconds()/1000000.0 > (execute_msg->hover_duration))) {
			loop_rate.sleep();
			RCLCPP_INFO(this->get_logger(), "Hovering in position succeeded.");
			break;
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_takeoff \n");

	rclcpp::Rate loop_rate(2);
	rclcpp::Rate loop_rate_arming(1);
	double takeoff_height = getHeightAMSL(execute_msg->altitude_level, execute_msg->goal_pose.position.altitude);
	auto result = std::make_shared<ExecuteSequence::Result>();

	if(is_flying_){

		return OffboardController::RETURN_VALUE::action_completed;

	}else{
		fc_interface_->publish_position_control_mode(); // To be able to arm

		int retry_count = 0;
		int max_retry_count = int(50/1);

		while(!status_listener_->get_arming_state() && retry_count < max_retry_count){

			fc_interface_->arm();

			if (goal_handle->is_canceling()) {
				fc_interface_->disarm();
				RCLCPP_INFO(this->get_logger(), "Takeoff canceled");
				return OffboardController::RETURN_VALUE::goal_canceled;
			}

			if(checkForExternalCommand()){
				fc_interface_->publish_position_control_mode();
				return OffboardController::RETURN_VALUE::action_paused;
			}

			loop_rate_arming.sleep();
			retry_count++;
		}

		if (retry_count >= max_retry_count){
			RCLCPP_INFO(this->get_logger(), "Takeoff aborted: [could not arm]");
			goal_handle->abort(result);
			return OffboardController::RETURN_VALUE::action_critical_failure;
		}

		RCLCPP_INFO(this->get_logger(),"Taking off to altitude: %f",takeoff_height);

		fc_interface_->takeoff(takeoff_height+15);
	}

	while (rclcpp::ok()) {

		loop_rate.sleep();

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_INFO(this->get_logger(), "TakeOff canceled");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && position_listener_->get_recent_gps_msg()->absolute_altitude_m > takeoff_height) {
			loop_rate.sleep();
			is_flying_ = true;
			this->position_listener_->set_recent_platform_state("AIRBORNE");
			RCLCPP_INFO(this->get_logger(), "Takeoff succeeded");
			break;
		}
	}
	fc_interface_->publish_position_control_mode();
	return OffboardController::RETURN_VALUE::action_completed;
}


int OffboardController::execute_landing(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_landing \n");
	rclcpp::Rate loop_rate(5);
	double land_height = -3.0;
	land_height = position_listener_->get_recent_home_msg()->absolute_altitude_m - 0.5;

	auto result = std::make_shared<ExecuteSequence::Result>();

	if(!is_flying_){
		return OffboardController::RETURN_VALUE::action_completed;
	}else{
		fc_interface_->land(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, land_height);
	}

	// Track position for stability check (to detect actual landing even if home altitude is inaccurate)
	auto last_position_check_time = this->get_clock()->now();
	double last_altitude = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
	double position_stability_threshold = 0.3; // 30 cm
	auto position_stability_duration = rclcpp::Duration::from_seconds(10.0); // 10 seconds

	while (rclcpp::ok()) {
		loop_rate.sleep();

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_INFO(this->get_logger(), "Landing canceled");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		double current_altitude = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
		auto current_time = this->get_clock()->now();

		// Primary check: altitude below home position
		if (rclcpp::ok() && current_altitude < (position_listener_->get_recent_home_msg()->absolute_altitude_m + 0.1)) {
			is_flying_ = false;
			this->position_listener_->set_recent_platform_state("LANDED");
			loop_rate.sleep();
			RCLCPP_INFO(this->get_logger(), "Landing detected via altitude check");
			break;
		}

		// Secondary check: position stability (for cases where home altitude is inaccurate)
		double altitude_change = std::abs(current_altitude - last_altitude);
		if (altitude_change > position_stability_threshold) {
			// Position changed significantly, reset timer
			last_position_check_time = current_time;
			last_altitude = current_altitude;
		} else {
			// Check if position has been stable for the required duration
			if ((current_time - last_position_check_time) >= position_stability_duration) {
				is_flying_ = false;
				this->position_listener_->set_recent_platform_state("LANDED");
				loop_rate.sleep();
				RCLCPP_INFO(this->get_logger(), "Landing detected via position stability check (altitude stable at %.2f m for 10 seconds)", current_altitude);
				break;
			}
		}
	}

	int retry_count = 0;
	int max_retry_count = int(50/1);
	rclcpp::Rate loop_rate_arming(1);

	while(status_listener_->get_arming_state() && retry_count < max_retry_count){

		fc_interface_->disarm();

		if (goal_handle->is_canceling()) {
			RCLCPP_INFO(this->get_logger(), "Landing canceled");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			fc_interface_->publish_position_control_mode();
			return OffboardController::RETURN_VALUE::action_paused;
		}

		loop_rate_arming.sleep();
		retry_count++;
	}

	RCLCPP_INFO(this->get_logger(), "Landing succeeded");

	fc_interface_->publish_position_control_mode(); // To be able to arm again

	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_stopCapture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_stopCapture\n");
	this->camera_controller_->stop_capture();
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_startCapture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_startCapture\n");
	this->camera_controller_->start_capture();
	return OffboardController::RETURN_VALUE::action_completed;
}

//Will turn in a 360 degree and trigger the camera to publish images
int OffboardController::execute_PanoramaPicture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg ){
	return OffboardController::RETURN_VALUE::action_completed;
}

//Will turn in a 360 degree and trigger the camera to publish images
int OffboardController::execute_releaseObject(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg ){
	RCLCPP_INFO(this->get_logger(),"In execute_releaseObject\n");

	if(!is_flying_){
		RCLCPP_ERROR(this->get_logger(),"Cannot release object while not flying!\n");
		return OffboardController::RETURN_VALUE::action_failure;
	}

	if(payload_ != ""){
		// Open dropper mechanism
		this->fc_interface_->set_servo_value(5,2000);

		rclcpp::Rate sleep_timer(2);
		sleep_timer.sleep();

		// Close dropper mechanism
		this->fc_interface_->set_servo_value(5,1000);

		RCLCPP_INFO(this->get_logger(),"Object of type %s released.\n", payload_.c_str());
	}else{
		RCLCPP_INFO(this->get_logger(),"No payload attached, skipping release.\n");
	}

	return OffboardController::RETURN_VALUE::action_completed;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////		Action Methods END	/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Generates a vector of atoms from a vector of gps positions for the fly motion
std::vector<ExecuteAtom> OffboardController::generateFlyAtomFromVector(std::vector<Vector3D> &gps_positions){
	std::vector<ExecuteAtom> atoms;

	for(auto& positions : gps_positions){
		ExecuteAtom atom;
		atom.action_type = "fly_step_3D";
		atom.goal_pose.position.latitude = positions.getX();
		atom.goal_pose.position.longitude = positions.getY();
		atom.goal_pose.position.altitude = positions.getZ();
		atom.altitude_level.value = AltitudeLevel::AMSL;
		atoms.push_back(atom);
	}
	return atoms;
}


void OffboardController::publish_position_feedback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle){

	auto feedback = std::make_shared<ExecuteSequence::Feedback>();
	feedback->current_pose.header.stamp = this->get_clock()->now();
	feedback->current_pose.pose.position.x = position_listener_->get_recent_gps_msg()->latitude_deg;
	feedback->current_pose.pose.position.y = position_listener_->get_recent_gps_msg()->longitude_deg;
	feedback->current_pose.pose.position.z = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
	feedback->finished_actions_in_sequence = this->current_action_index;
	goal_handle->publish_feedback(feedback);

}

void OffboardController::get_origin(const std::shared_ptr<auspex_msgs::srv::GetOrigin::Request> request, std::shared_ptr<auspex_msgs::srv::GetOrigin::Response> response){

		response->origin.latitude = position_listener_->get_recent_home_msg()->latitude_deg;
		response->origin.longitude = position_listener_->get_recent_home_msg()->longitude_deg;
		response->origin.altitude = position_listener_->get_recent_gps_msg()->relative_altitude_m;
		response->success = true;
}

/**
* @brief callback for set_origin service --> actually just setting home. 
*/
void OffboardController::set_origin(const std::shared_ptr<auspex_msgs::srv::SetOrigin::Request> request, std::shared_ptr<auspex_msgs::srv::SetOrigin::Response> response){
	double lat = request->origin.latitude;
	double lon = request->origin.longitude;
	double alt = request->origin.altitude;

	
	//Storing the height delta for AUSPEX -> FC commands (FC height + offset = AUSPEX height)
	fc_interface_->set_height_delta(position_listener_->get_fc_height() - (alt + position_listener_->get_recent_gps_msg()->relative_altitude_m));

	// Setting the ground distance to have accurate FC -> AUSPEX heights (relative + ground height)
	position_listener_->set_home_ground_altitude_amsl(alt);
	
	//wait
	rclcpp::Rate sleep_timer(2);
	sleep_timer.sleep();
	
	// reset gps converter to use the new heights
	gps_converter_ = std::make_shared<GeodeticConverter>(position_listener_->get_recent_home_msg()->latitude_deg, position_listener_->get_recent_home_msg()->longitude_deg, position_listener_->get_recent_home_msg()->absolute_altitude_m);
	fc_interface_->set_gps_converter(gps_converter_);

	//check if home is new one
	response->success = false;
	double eps = 0.001;
	if(std::fabs(position_listener_->get_recent_home_msg()->latitude_deg - lat)< eps &&
		std::fabs(position_listener_->get_recent_home_msg()->longitude_deg - lon)< eps&&
		std::fabs(position_listener_->get_recent_home_msg()->absolute_altitude_m - alt)< eps){
		response->success = true;
	}

	RCLCPP_INFO(this->get_logger(), "New origin set to: %f, %f, %f", lat, lon, alt);
}

/**
*@brief Handles the add action call
*/
void OffboardController::handle_add_action(const std::shared_ptr<auspex_msgs::srv::AddAction::Request> request, std::shared_ptr<auspex_msgs::srv::AddAction::Response> response){
	RCLCPP_INFO(this->get_logger(), "Adding new actions to action list...");
	auto atoms_list = request->execute_atoms;

	execute_queue_lock.lock();

	for(auto &atom: atoms_list){
		this->executeSequenceQueue.push_front(atom);
	}

	execute_queue_lock.unlock();
	response->error_code = 0;
}

/**
*@brief Handles platform commands
*/
void OffboardController::handle_platform_command(const PlatformCommand::SharedPtr msg){
	if(msg->platform_command == PlatformCommand::PLATFORM_CANCEL){
		RCLCPP_ERROR(this->get_logger(), "Not Implemented!");
	}else if(msg->platform_command == PlatformCommand::PLATFORM_PAUSE){
		RCLCPP_INFO(this->get_logger(), "Requested Pause.");
		status_listener_->set_paused_from_extern(true);
		if(is_flying_){
			this->position_listener_->set_recent_platform_state("AIRBORNE - PAUSED");
		}else{
			this->position_listener_->set_recent_platform_state("PAUSED");
		}
	}else if(msg->platform_command == PlatformCommand::PLATFORM_RESUME){
		status_listener_->set_paused_from_extern(false);
		if(is_flying_){
			this->position_listener_->set_recent_platform_state("AIRBORNE");
		}else{
			this->position_listener_->set_recent_platform_state("LANDED");
		}
		RCLCPP_INFO(this->get_logger(), "Requested Resume.");
	}else if(msg->platform_command == PlatformCommand::PLATFORM_TERMINATE){
		RCLCPP_INFO(this->get_logger(), "Requested Terminate.");
		this->fc_interface_->terminate();
		rclcpp::sleep_for(std::chrono::seconds(4));
		rclcpp::shutdown();
	}else if(msg->platform_command == PlatformCommand::PLATFORM_KILL){
		RCLCPP_INFO(this->get_logger(), "Requested Kill.");
		this->fc_interface_->kill();
		rclcpp::sleep_for(std::chrono::seconds(4));
		rclcpp::shutdown();
	}else{
		RCLCPP_ERROR(this->get_logger(), "Unknown Platform Command.");
	}
}

bool OffboardController::checkForExternalCommand(){
	if(status_listener_->get_paused_from_extern()){
		return true;
	}else{
		return false;
	}
}

double OffboardController::get_groundHeight_amsl(){
	auto request = std::make_shared<GetAltitude::Request>();
	request->gps_position.latitude = position_listener_->get_recent_gps_msg()->latitude_deg;
	request->gps_position.longitude = position_listener_->get_recent_gps_msg()->longitude_deg;
	request->gps_position.altitude = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
	request->resolution = 1;

	double altitude = 0.0;
	// Send the service request asynchronously
	while (!this->get_altitude_client->wait_for_service(1s)) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_altitude_client service not available, waiting again...");
	}

	auto future_result = this->get_altitude_client->async_send_request(request);

	try {
		auto response = future_result.get();
		if (response->success) {
			altitude = response->altitude_amsl;
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to call service get_altitude");
		}
	} catch (const std::exception &e) {
		RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
	}
	return altitude;
}

double OffboardController::getHeightAMSL(AltitudeLevel level, double value){
	double height_amsl = 0.0;
	if(level.value == AltitudeLevel::AMSL){
		height_amsl = value;
	}else if (level.value == AltitudeLevel::AGL){
		// AGL
		height_amsl = (this->get_groundHeight_amsl() + value);

	}else{
		height_amsl = position_listener_->get_recent_gps_msg()->absolute_altitude_m + value;
	}
	return height_amsl;
}

std::vector<Vector3D> OffboardController::get_scanWaypoints(double scan_height, double sensor_footprint, std::vector<geometry_msgs::msg::Pose2D> polygon_vertices, ScanPattern pattern) {
    if (polygon_vertices.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Error: Not enough vertices specified for scanning (minimum 3 required).");
        return {};
    }

    switch (pattern) {
        case ScanPattern::LAWNMOWER:
            return generate_lawnmower_pattern(scan_height, sensor_footprint, polygon_vertices);
        case ScanPattern::SPIRAL:
            return generate_spiral_pattern(scan_height, sensor_footprint, polygon_vertices);
        default:
            RCLCPP_ERROR(this->get_logger(), "Error: Unknown scan pattern specified.");
            return {};
    }
}

geometry_msgs::msg::Pose2D OffboardController::find_closest_polygon_point(const std::vector<geometry_msgs::msg::Pose2D>& polygon_vertices) {
    auto current_pos = position_listener_->get_recent_gps_msg();
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Pose2D closest_point;
    
    for (const auto& vertex : polygon_vertices) {
        double dist = gps_converter_->geodeticDistance2D(
            current_pos->latitude_deg, current_pos->longitude_deg,
            vertex.x, vertex.y
        );
        if (dist < min_distance) {
            min_distance = dist;
            closest_point = vertex;
        }
    }
    
    return closest_point;
}

std::vector<Vector3D> OffboardController::generate_lawnmower_pattern(double scan_height, double sensor_footprint, std::vector<geometry_msgs::msg::Pose2D> polygon_vertices) {
    RCLCPP_INFO(this->get_logger(), "Generating lawnmower scan pattern");
    
    // Calculate bounding box of the polygon
    double lat_min = std::numeric_limits<double>::max();
    double lat_max = std::numeric_limits<double>::lowest();
    double lon_min = std::numeric_limits<double>::max(); 
    double lon_max = std::numeric_limits<double>::lowest();
    
    for (const auto& vertex : polygon_vertices) {
        lat_min = std::min(lat_min, vertex.x);
        lat_max = std::max(lat_max, vertex.x);
        lon_min = std::min(lon_min, vertex.y);
        lon_max = std::max(lon_max, vertex.y);
    }
    
    // Find closest corner to current position
    auto current_pos = position_listener_->get_recent_gps_msg();
    std::vector<std::pair<double, double>> corners = {
        {lat_min, lon_min}, {lat_min, lon_max}, {lat_max, lon_max}, {lat_max, lon_min}
    };
    
    double min_distance = std::numeric_limits<double>::max();
    int start_corner = 0;
    
    for (size_t i = 0; i < corners.size(); ++i) {
        double dist = gps_converter_->geodeticDistance2D(
            current_pos->latitude_deg, current_pos->longitude_deg,
            corners[i].first, corners[i].second
        );
        if (dist < min_distance) {
            min_distance = dist;
            start_corner = static_cast<int>(i);
        }
    }
    
    // Calculate scan line spacing in degrees (approximately)
    double lat_spacing = sensor_footprint / 111320.0; // 1 degree latitude ≈ 111.32 km
    double lon_spacing = sensor_footprint / (111320.0 * cos(lat_min * M_PI / 180.0)); // Account for longitude convergence
    
    // Determine scan direction based on bounding box aspect ratio
    double lat_span = lat_max - lat_min;
    double lon_span = lon_max - lon_min;
    bool scan_horizontally = (lon_span > lat_span);
    
    std::vector<Vector3D> waypoints;
    
    if (scan_horizontally) {
        // Scan horizontally (east-west lines)
        double current_lat = (start_corner < 2) ? lat_min : lat_max;
        bool going_east = (start_corner == 0 || start_corner == 3);
        int direction = going_east ? 1 : -1;
        
        int num_lines = static_cast<int>(std::ceil(lat_span / lat_spacing)) + 1;
        
        for (int i = 0; i < num_lines; ++i) {
            double line_lat = (start_corner < 2) ? (lat_min + i * lat_spacing) : (lat_max - i * lat_spacing);
            line_lat = std::max(lat_min, std::min(lat_max, line_lat));
            
            if (direction > 0) {
                waypoints.emplace_back(line_lat, lon_min, scan_height);
                waypoints.emplace_back(line_lat, lon_max, scan_height);
            } else {
                waypoints.emplace_back(line_lat, lon_max, scan_height);
                waypoints.emplace_back(line_lat, lon_min, scan_height);
            }
            direction *= -1; // Alternate direction for lawnmower pattern
        }
    } else {
        // Scan vertically (north-south lines)
        double current_lon = (start_corner == 0 || start_corner == 1) ? lon_min : lon_max;
        bool going_north = (start_corner == 0 || start_corner == 3);
        int direction = going_north ? 1 : -1;
        
        int num_lines = static_cast<int>(std::ceil(lon_span / lon_spacing)) + 1;
        
        for (int i = 0; i < num_lines; ++i) {
            double line_lon = (start_corner == 0 || start_corner == 1) ? (lon_min + i * lon_spacing) : (lon_max - i * lon_spacing);
            line_lon = std::max(lon_min, std::min(lon_max, line_lon));
            
            if (direction > 0) {
                waypoints.emplace_back(lat_min, line_lon, scan_height);
                waypoints.emplace_back(lat_max, line_lon, scan_height);
            } else {
                waypoints.emplace_back(lat_max, line_lon, scan_height);
                waypoints.emplace_back(lat_min, line_lon, scan_height);
            }
            direction *= -1; // Alternate direction for lawnmower pattern
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for lawnmower pattern", waypoints.size());
    return waypoints;
}

std::vector<Vector3D> OffboardController::generate_spiral_pattern(double scan_height, double sensor_footprint, std::vector<geometry_msgs::msg::Pose2D> polygon_vertices) {
    RCLCPP_INFO(this->get_logger(), "Generating spiral scan pattern");
    
    // Calculate bounding box and center
    double lat_min = std::numeric_limits<double>::max();
    double lat_max = std::numeric_limits<double>::lowest();
    double lon_min = std::numeric_limits<double>::max();
    double lon_max = std::numeric_limits<double>::lowest();
    
    for (const auto& vertex : polygon_vertices) {
        lat_min = std::min(lat_min, vertex.x);
        lat_max = std::max(lat_max, vertex.x);
        lon_min = std::min(lon_min, vertex.y);
        lon_max = std::max(lon_max, vertex.y);
    }
    
    double center_lat = (lat_min + lat_max) / 2.0;
    double center_lon = (lon_min + lon_max) / 2.0;
    
    // Find closest point to current position to determine spiral direction
    auto current_pos = position_listener_->get_recent_gps_msg();
    geometry_msgs::msg::Pose2D closest_corner = find_closest_polygon_point(polygon_vertices);
    
    // Calculate maximum radius needed to cover the area
    double max_radius_m = 0.0;
    std::vector<std::pair<double, double>> corners = {
        {lat_min, lon_min}, {lat_min, lon_max}, {lat_max, lon_max}, {lat_max, lon_min}
    };
    
    for (const auto& corner : corners) {
        double dist = gps_converter_->geodeticDistance2D(
            center_lat, center_lon, corner.first, corner.second
        );
        max_radius_m = std::max(max_radius_m, dist);
    }
    
    std::vector<Vector3D> waypoints;
    
    // Generate spiral waypoints
    double radius_step = sensor_footprint * 0.8; // Overlap for complete coverage
    double angle_step = M_PI / 8.0; // 22.5 degree steps
    int num_spirals = static_cast<int>(std::ceil(max_radius_m / radius_step));
    
    // Determine if we should spiral inward or outward based on closest point
    double dist_to_center = gps_converter_->geodeticDistance2D(
        current_pos->latitude_deg, current_pos->longitude_deg,
        center_lat, center_lon
    );
    bool spiral_inward = (dist_to_center > max_radius_m * 0.7); // Start from outside if currently far from center
    
    if (spiral_inward) {
        // Start from outside and spiral inward
        for (int spiral = num_spirals; spiral >= 1; --spiral) {
            double radius = spiral * radius_step;
            int points_per_circle = std::max(8, static_cast<int>(2 * M_PI * radius / sensor_footprint));
            double angle_increment = 2 * M_PI / points_per_circle;
            
            for (int point = 0; point < points_per_circle; ++point) {
                double angle = point * angle_increment;
                
                // Convert radius and angle to lat/lon offset
                double lat_offset = (radius * cos(angle)) / 111320.0;
                double lon_offset = (radius * sin(angle)) / (111320.0 * cos(center_lat * M_PI / 180.0));
                
                double waypoint_lat = center_lat + lat_offset;
                double waypoint_lon = center_lon + lon_offset;
                
                // Check if waypoint is within bounding box
                if (waypoint_lat >= lat_min && waypoint_lat <= lat_max &&
                    waypoint_lon >= lon_min && waypoint_lon <= lon_max) {
                    waypoints.emplace_back(waypoint_lat, waypoint_lon, scan_height);
                }
            }
        }
    } else {
        // Start from center and spiral outward
        waypoints.emplace_back(center_lat, center_lon, scan_height); // Start at center
        
        for (int spiral = 1; spiral <= num_spirals; ++spiral) {
            double radius = spiral * radius_step;
            int points_per_circle = std::max(8, static_cast<int>(2 * M_PI * radius / sensor_footprint));
            double angle_increment = 2 * M_PI / points_per_circle;
            
            for (int point = 0; point < points_per_circle; ++point) {
                double angle = point * angle_increment;
                
                // Convert radius and angle to lat/lon offset
                double lat_offset = (radius * cos(angle)) / 111320.0;
                double lon_offset = (radius * sin(angle)) / (111320.0 * cos(center_lat * M_PI / 180.0));
                
                double waypoint_lat = center_lat + lat_offset;
                double waypoint_lon = center_lon + lon_offset;
                
                // Check if waypoint is within bounding box
                if (waypoint_lat >= lat_min && waypoint_lat <= lat_max &&
                    waypoint_lon >= lon_min && waypoint_lon <= lon_max) {
                    waypoints.emplace_back(waypoint_lat, waypoint_lon, scan_height);
                }
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for spiral pattern", waypoints.size());
    return waypoints;
}