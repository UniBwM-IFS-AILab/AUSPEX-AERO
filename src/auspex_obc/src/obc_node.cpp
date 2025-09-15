//Load header files
#include"auspex_obc/obc_node.h"

#include <sstream> // For std::istringstream


OffboardController::OffboardController(	std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener,
										std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener,
										std::shared_ptr<FC_Interface_Base> fc_interface,
										std::shared_ptr<DroneStatePublisher> drone_state_publisher,
										std::shared_ptr<CamPublisher> cam_publisher,
										std::string name_prefix) : Node(name_prefix+ "_" + "offboard_control_node") {

	name_prefix_ = name_prefix;
	vehicle_status_listener_ = vehicle_status_listener;
	position_listener_ = position_listener;
	camera_publisher_ = cam_publisher;
	drone_state_publisher_ = drone_state_publisher;
	fc_interface_ = fc_interface;

	this->position_listener_->set_recent_platform_state("INITIALIZING");

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	handle_user_command_listener_ =  this->create_subscription<UserCommand>(name_prefix + "/user_command", qos, std::bind(&OffboardController::handle_user_command, this, _1));

	gps_converter_ = std::make_shared<GeodeticConverter>(position_listener_->get_recent_home_msg()->latitude_deg, position_listener_->get_recent_home_msg()->longitude_deg, position_listener_->get_recent_home_msg()->absolute_altitude_m);
	fc_interface_->set_gps_converter(gps_converter_);
	RCLCPP_INFO(this->get_logger(), "Set GPS Origin to configured FC origin (%lf, %lf, %lf)", position_listener_->get_recent_home_msg()->latitude_deg, position_listener_->get_recent_home_msg()->longitude_deg, position_listener_->get_recent_home_msg()->absolute_altitude_m);

	timer_ = this->create_wall_timer(250ms, std::bind(&OffboardController::publish_heartbeat, this));

	this->start_action_server();
	this->start_services();

	this->get_altitude_client = this->create_client<GetAltitude>("/auspex_get_altitude");

	int retry_count = 0;
	is_height_data_available = true;

	while (!this->get_altitude_client->wait_for_service(1s)) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_altitude_client service not available, waiting again...");
		retry_count++;
		if(retry_count > 3){
			is_height_data_available = false;
			break;
		}
	}

	this->drone_state_publisher_->start_publish(this->position_listener_, this->vehicle_status_listener_);
	this->camera_publisher_->startCapture(this->position_listener_);
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
		name_prefix_ + "/action_sequence",
		std::bind(&OffboardController::handle_goal, this, _1, _2),
		std::bind(&OffboardController::handle_cancel_sequence, this, _1),
		std::bind(&OffboardController::handle_accepted_sequence, this, _1));
		RCLCPP_INFO(get_logger(), "Action servers are ready. \n");
}

void OffboardController::start_services(){
	using namespace std::placeholders;

	RCLCPP_INFO(this->get_logger(), "Starting services...");
	get_origin_service_ = this -> create_service<auspex_msgs::srv::GetOrigin>(name_prefix_ + "/srv/get_origin", std::bind(&OffboardController::get_origin, this, _1, _2));
	set_origin_service_ = this -> create_service<auspex_msgs::srv::SetOrigin>(name_prefix_ + "/srv/set_origin", std::bind(&OffboardController::set_origin, this, _1, _2));
	handle_add_action_service_ = this -> create_service<auspex_msgs::srv::AddAction>(name_prefix_ + "/srv/add_action", std::bind(&OffboardController::handle_add_action, this, _1, _2));
	RCLCPP_INFO(this->get_logger(), "Services are ready.");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////	Action Methods BEGIN	/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OffboardController::execute_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle){
	RCLCPP_INFO(this->get_logger(),"In Execute Sequence" );
	vehicle_status_listener_->set_paused_from_extern(false);

	auto executeAtomsVector = goal_handle->get_goal()->execute_atoms;
	auto result = std::make_shared<ExecuteSequence::Result>();
	executeSequenceQueue.clear();

	for(auto &atom: executeAtomsVector){
		executeSequenceQueue.emplace_back(atom);
	}

	if(executeSequenceQueue.empty()){
		RCLCPP_ERROR(this->get_logger(),"ERROR: Received empty execute atoms list.");
		return;
	}

	current_action_index = 0;
	int initial_action_count = executeAtomsVector.size();
	int return_code = OffboardController::RETURN_VALUE::action_critical_failure;

	rclcpp::Rate loop_rate(10);
	rclcpp::Rate loop_rate_slow(1);

	ExecuteAtom atom;

	while(!executeSequenceQueue.empty()){
		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Action Sequence canceled");
			return;
		}

		if(vehicle_status_listener_->get_paused_from_extern()){
			loop_rate.sleep();
			publish_position_feedback(goal_handle);
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
			return_code = execute_takeoff(goal_handle, &atom);
		else if(atom.action_type == "land")
			return_code = execute_landing(goal_handle, &atom);
		else if(atom.action_type == "fly_3D")
			return_code = execute_waypoint3D(goal_handle, &atom);
		else if(atom.action_type == "hover")
			return_code = execute_hover(goal_handle, &atom);
		else if(atom.action_type == "scan_area_uav")
			return_code = execute_scanArea(goal_handle, &atom);
		else if(atom.action_type == "ascend")
			return_code = execute_ascend(goal_handle, &atom);
		else if(atom.action_type == "descend")
			return_code = execute_descend(goal_handle, &atom);
		else if(atom.action_type == "fly_above_highest_point")
			return_code = execute_flyAboveHeighestPoint(goal_handle, &atom);
		else if(atom.action_type == "fly_2D")
			return_code = execute_waypoint2D(goal_handle, &atom);
		else if(atom.action_type == "fly_step_3D")
			return_code = execute_waypoint3D_step(goal_handle, &atom);
		else if(atom.action_type == "search_area_uav")
			return_code = execute_searchArea(goal_handle, &atom);
		else if(atom.action_type == "start_detection")
			return_code = execute_startCapture(goal_handle, &atom);
		else if(atom.action_type == "stop_detection")
			return_code = execute_stopCapture(goal_handle, &atom);
		else if(atom.action_type == "capture_image")
			return_code = execute_takeImage(goal_handle, &atom);
		else if(atom.action_type == "circle_poi")
			return_code = execute_CirclePoI(goal_handle, &atom);
		else if(atom.action_type == "hover_right")
			return_code = execute_hoverRight(goal_handle, &atom);
		else if(atom.action_type == "hover_left")
			return_code = execute_hoverLeft(goal_handle, &atom);
		else if(atom.action_type == "turn")
			return_code = execute_turn_by_angle(goal_handle, &atom);
		else if(atom.action_type == "fly_at_ground_distance")
			return_code = execute_flyDistance2Ground(goal_handle, &atom);
		else if(atom.action_type == "return_home_and_land")
			return_code = execute_flyHomeAndLand(goal_handle, &atom);
		else{
			RCLCPP_ERROR(this->get_logger(), "Unknown action type: %s", atom.action_type.c_str());
			return_code = OffboardController::RETURN_VALUE::action_unkown;
		}

		//Locks the queue checks if it was the last action in it and removes it from the queue.
		execute_queue_lock.lock();
		bool final_action = false;
		if(return_code != OffboardController::RETURN_VALUE::action_paused && return_code != OffboardController::RETURN_VALUE::action_splitted ){
			final_action = executeSequenceQueue.size() == 1;
			executeSequenceQueue.pop_front();
		}
		execute_queue_lock.unlock();


		switch (return_code) {
			case OffboardController::RETURN_VALUE::action_completed:

				current_action_index++;
				RCLCPP_INFO(this->get_logger(), "Action %d out of %ld in sequence successfully completed", current_action_index, initial_action_count);
				if(final_action){
					RCLCPP_INFO(this->get_logger(), "Sequence successfully completed");
					goal_handle->succeed(result);
				}
				break;

			case OffboardController::RETURN_VALUE::goal_canceled:

				RCLCPP_INFO(this->get_logger(), "Sequence canceled during action %d out of %ld",(current_action_index+1), initial_action_count);
				RCLCPP_INFO(this->get_logger(), "Successfully completed %d actions of the sequence",(current_action_index+1));
				this->publish_position_feedback(goal_handle);
				goal_handle->canceled(result);
				return ;

			case OffboardController::RETURN_VALUE::action_paused:{

				RCLCPP_INFO(this->get_logger(), "Action Paused from extern. Waiting for resume...");
				this->camera_publisher_->stopCapture();
				break;

			}
			case OffboardController::RETURN_VALUE::action_critical_failure:{

				RCLCPP_INFO(this->get_logger(), "Recent Action failed critically. Engaging failsafe mode");
				if(is_flying_){
					fc_interface_->publish_position_control_mode();
				}
				this->publish_position_feedback(goal_handle);
				goal_handle->canceled(result);
				return ;

			}
			case OffboardController::RETURN_VALUE::action_unkown:{

				RCLCPP_INFO(this->get_logger(), "Unknown Action type in sequence. Returning...");
				if(is_flying_){
					fc_interface_->publish_position_control_mode();
				}
				this->publish_position_feedback(goal_handle);
				goal_handle->canceled(result);
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

	this->publish_position_feedback(goal_handle);
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

	double distance = 0.0;
	double max_distance = 30;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = execute_msg->goal_pose;
		double current_distance = gps_converter_->geodeticDistance(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

		if (current_distance > max_distance) {
			double fraction = max_distance / current_distance;
			intermediate_goal.position.latitude = position_listener_->get_recent_gps_msg()->latitude_deg + fraction * (intermediate_goal.position.latitude -  position_listener_->get_recent_gps_msg()->latitude_deg);
    		intermediate_goal.position.longitude = position_listener_->get_recent_gps_msg()->longitude_deg + fraction * (intermediate_goal.position.longitude - position_listener_->get_recent_gps_msg()->longitude_deg);
		}

		distance = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, position_listener_->get_recent_gps_msg()->absolute_altitude_m, HEADING::UNCHANGED, execute_msg->speed_m_s);//alt amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance <= 0.5) {
			loop_rate.sleep();
			RCLCPP_DEBUG(this->get_logger(), "Hovering left to current waypoint succeeded");
			break;
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

	double distance = 0.0;
	double max_distance = 30.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = execute_msg->goal_pose;
		double current_distance = gps_converter_->geodeticDistance(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

		if (current_distance > max_distance) {
			double fraction = max_distance / current_distance;
			intermediate_goal.position.latitude = position_listener_->get_recent_gps_msg()->latitude_deg + fraction * (intermediate_goal.position.latitude -  position_listener_->get_recent_gps_msg()->latitude_deg);
    		intermediate_goal.position.longitude = position_listener_->get_recent_gps_msg()->longitude_deg + fraction * (intermediate_goal.position.longitude - position_listener_->get_recent_gps_msg()->longitude_deg);
		}

		distance = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, position_listener_->get_recent_gps_msg()->absolute_altitude_m, HEADING::UNCHANGED, execute_msg->speed_m_s);//alt above mean sea level

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance <= 0.5) {
			loop_rate.sleep();
			RCLCPP_DEBUG(this->get_logger(), "hovering right to current waypoint succeeded");
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
		return OffboardController::RETURN_VALUE::action_paused;
	}

	double goal_height = execute_msg->goal_pose.position.altitude;
	switch (execute_msg->altitude_level.value) {
		case AltitudeLevel::AMSL:
			goal_height -= position_listener_->get_recent_home_msg()->absolute_altitude_m;
			break;
		case AltitudeLevel::AGL:
			if (is_height_data_available) {
				goal_height = (get_groundHeight_amsl() + goal_height) - position_listener_->get_recent_home_msg()->absolute_altitude_m;
			} else {
				RCLCPP_WARN(this->get_logger(), "Height data not available... flying above ground not recommended.");
			}
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

	this->execute_startCapture(goal_handle, execute_msg);

	while (rclcpp::ok()) {
		loop_rate.sleep();

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_INFO(this->get_logger(), "Canceled Circle POI");
			this->execute_stopCapture(goal_handle, execute_msg);
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			this->execute_stopCapture(goal_handle, execute_msg);
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
	this->execute_stopCapture(goal_handle, execute_msg);
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

	//the sensor FOV in degree
	double sensor_fov_hor = 30;// 30 degree means +/- 15
	double sensor_fov_vert = 30;

	//compute footprint
	double sensor_height = (tan(((sensor_fov_hor/2.0)*M_PI)/180.0) * scan_height) * 2.0;//in metres
	double sensor_width = (tan(((sensor_fov_vert/2.0)*M_PI)/180.0) * scan_height) * 2.0;//in metres

	//abort if there are not enough vertices given
	if(polygon_vertices.size() < 3){
		RCLCPP_ERROR(this->get_logger(), "Error: Not enough vertices specified for scanning.");
		return OffboardController::RETURN_VALUE::action_failure;
	}
	std::vector<Vector3D> scanWaypoints = this->get_scanWaypoints(execute_msg->height, sensor_width, polygon_vertices);
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
    constexpr double SENSOR_FOV_VERT = 30.0;

    // Compute sensor footprint in meters
    double sensor_width = 2.0 * tan((SENSOR_FOV_HOR / 2.0) * M_PI / 180.0) * scan_height;
    double sensor_height = 2.0 * tan((SENSOR_FOV_VERT / 2.0) * M_PI / 180.0) * scan_height;

    if (polygon_vertices.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Error: Not enough vertices specified for scanning.");
        return OffboardController::RETURN_VALUE::action_failure;
    }

    std::vector<Vector3D> scanWaypoints = this->get_scanWaypoints(execute_msg->height, sensor_width, polygon_vertices);
    std::vector<ExecuteAtom> executeAtoms_wp = this->generateFlyAtomFromVector(scanWaypoints);
    RCLCPP_INFO(this->get_logger(), "Searching for object at height: %f meters", scan_height);
    this->execute_startCapture(goal_handle, execute_msg);

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
            this->execute_stopCapture(goal_handle, execute_msg);
            return result_atom;
        }
        else {
            this->execute_stopCapture(goal_handle, execute_msg);
            return result_atom;
        }
    }

    this->execute_stopCapture(goal_handle, execute_msg);
    return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_waypoint3D_step(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){

	RCLCPP_INFO(this->get_logger(),"In execute_waypoint 2 Steps \n");

	geographic_msgs::msg::GeoPose pose;
	pose = execute_msg->goal_pose;
	double current_alt = position_listener_->get_recent_gps_msg()->absolute_altitude_m;
	double goal_alt = pose.position.altitude; //amsl
	int result_atom = 0;

	if(current_alt > goal_alt){
		result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			result_atom = this->execute_descend(goal_handle, execute_msg);
		}
	}else if (current_alt < goal_alt){
		result_atom = this->execute_ascend(goal_handle, execute_msg);
		if(result_atom == OffboardController::RETURN_VALUE::action_completed){
			result_atom = this->execute_waypoint2D(goal_handle, execute_msg);
		}
	}else{
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

	double goal_height = getHeightAMSL(execute_msg->altitude_level, execute_msg->goal_pose.position.altitude);

	RCLCPP_INFO(this->get_logger(), "Ascending from %f to %f .", position_listener_->get_recent_gps_msg()->absolute_altitude_m, goal_height);

	rclcpp::Rate loop_rate(5);
	auto result = std::make_shared<ExecuteSequence::Result>();
	double distance = 0.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		distance = fc_interface_->move_to_gps(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, goal_height, HEADING::UNCHANGED, execute_msg->speed_m_s); // goal_height in amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance <= 0.5) {
			loop_rate.sleep();
			RCLCPP_DEBUG(this->get_logger(), "Acending to current waypoint succeeded");
			break;
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
	double distance = 0.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		distance = fc_interface_->move_to_gps(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, goal_height, HEADING::UNCHANGED, execute_msg->speed_m_s);//alt should be amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance <= 0.5) {
			loop_rate.sleep();
			break;
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
	double distance = 0.0;
	double max_distance = 15.0;

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = pose;
		double current_distance = gps_converter_->geodeticDistance(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

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
		distance = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, pose.position.altitude, heading, execute_msg->speed_m_s); //alt is in amsl

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance <= 0.5) {
			loop_rate.sleep();
			break;
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
	double distance = 0.0;
	double max_distance = 15.0; //metres

	while (rclcpp::ok()) {
		loop_rate.sleep();
		geographic_msgs::msg::GeoPose intermediate_goal = pose;
		double current_distance = gps_converter_->geodeticDistance(position_listener_->get_recent_gps_msg()->latitude_deg, position_listener_->get_recent_gps_msg()->longitude_deg, intermediate_goal.position.latitude, intermediate_goal.position.longitude);

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
		distance = fc_interface_->move_to_gps(intermediate_goal.position.latitude, intermediate_goal.position.longitude, goal_height, heading, execute_msg->speed_m_s); //alt should be above mean sea level

		if (goal_handle->is_canceling()) {
			if(is_flying_){
				fc_interface_->publish_position_control_mode();
			}
			RCLCPP_DEBUG(this->get_logger(), "Navigation to current waypoint canceled");
			return OffboardController::RETURN_VALUE::goal_canceled;
		}

		if(checkForExternalCommand()){
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && distance <= 0.5) {
			loop_rate.sleep();
			RCLCPP_DEBUG(this->get_logger(), "Navigation to current waypoint succeeded");
			break;
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

		int retry_count = 0;
		int max_retry_count = int(50/1);

		while(!vehicle_status_listener_->get_arming_state() && retry_count < max_retry_count){

			fc_interface_->arm();

			if (goal_handle->is_canceling()) {
				fc_interface_->disarm();
				RCLCPP_INFO(this->get_logger(), "Takeoff canceled");
				return OffboardController::RETURN_VALUE::goal_canceled;
			}

			if(checkForExternalCommand()){
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
			return OffboardController::RETURN_VALUE::action_paused;
		}

		if (rclcpp::ok() && position_listener_->get_recent_gps_msg()->absolute_altitude_m < (position_listener_->get_recent_home_msg()->absolute_altitude_m + 0.1)) {
			is_flying_ = false;
			this->position_listener_->set_recent_platform_state("LANDED");
			loop_rate.sleep();
			RCLCPP_INFO(this->get_logger(), "Landing succeeded");
			break;
		}
	}
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_stopCapture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_stopCapture\n");
	this->camera_publisher_->stopCapture();
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_startCapture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_startCapture\n");
	this->camera_publisher_->startCapture(this->position_listener_);
	return OffboardController::RETURN_VALUE::action_completed;
}

int OffboardController::execute_takeImage(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg){
	RCLCPP_INFO(this->get_logger(),"In execute_takeImage\n");
	this->camera_publisher_->captureFrame();
	return OffboardController::RETURN_VALUE::action_completed;
}

//Will turn in a 360 degree and trigger the camera to publish images
int OffboardController::execute_PanoramaPicture(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteSequence>> goal_handle, ExecuteAtom* execute_msg ){
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
		response->origin.altitude = position_listener_->get_recent_home_msg()->absolute_altitude_m;
		response->success = true;
}

/**
* @brief callback for set_origin service --> actually just setting home. NED origin should stay the same
*/
void OffboardController::set_origin(const std::shared_ptr<auspex_msgs::srv::SetOrigin::Request> request, std::shared_ptr<auspex_msgs::srv::SetOrigin::Response> response){
	double lat = request->origin.latitude;
	double lon = request->origin.longitude;
	double alt = request->origin.altitude;

	RCLCPP_INFO(this->get_logger(), "Setting new origin to : lat: %f;long: %f;alt: %f", lat, lon, alt);

	//send new home to FC
	fc_interface_->set_home_fc(lat, lon, alt);

	//wait
	rclcpp::Rate sleep_timer(2);
	sleep_timer.sleep();

	//check if home is new one
	response->success = false;
	double eps = 0.001;
	if(std::fabs(position_listener_->get_recent_home_msg()->latitude_deg - lat)< eps &&
		std::fabs(position_listener_->get_recent_home_msg()->longitude_deg - lon)< eps&&
		std::fabs(position_listener_->get_recent_home_msg()->absolute_altitude_m - alt)< eps){

		response->success = true;
	}
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
*@brief Handles user commands NOT IMPLEMENTED
*/
void OffboardController::handle_user_command(const UserCommand::SharedPtr msg){
	if(msg->user_command == UserCommand::USER_CANCEL){
		RCLCPP_ERROR(this->get_logger(), "Not Implemented!");
	}else if(msg->user_command == UserCommand::USER_PAUSE){
		RCLCPP_INFO(this->get_logger(), "User Requested Pause.");
		vehicle_status_listener_->set_paused_from_extern(true);
		if(is_flying_){
			this->position_listener_->set_recent_platform_state("AIRBORNE - PAUSED");
		}else{
			this->position_listener_->set_recent_platform_state("PAUSED");
		}
	}else if(msg->user_command == UserCommand::USER_RESUME){
		vehicle_status_listener_->set_paused_from_extern(false);
		if(is_flying_){
			this->position_listener_->set_recent_platform_state("AIRBORNE");
		}else{
			this->position_listener_->set_recent_platform_state("LANDED");
		}
		RCLCPP_INFO(this->get_logger(), "User Requested Resume.");
	}else{
		RCLCPP_INFO(this->get_logger(), "Got Command which is not implemented");
	}
}

bool OffboardController::checkForExternalCommand(){
	if(vehicle_status_listener_->get_paused_from_extern()){
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
		if(is_height_data_available){
			height_amsl = (this->get_groundHeight_amsl() + value);
		}else{
			RCLCPP_INFO(this->get_logger(), "Height data not available... flying above ground not recommended.");
			height_amsl = value;
		}
	}else{
		height_amsl = position_listener_->get_recent_gps_msg()->absolute_altitude_m + value;
	}
	return height_amsl;
}

std::vector<Vector3D> OffboardController::get_scanWaypoints(double scan_height, double sensor_width, std::vector<geometry_msgs::msg::Pose2D> polygon_vertices){
    // Initialize bounding box values
    double x_min = 91.0, x_max = -91.0;  // Latitude range
    double y_min = 181.0, y_max = -181.0; // Longitude range

    // Compute bounding box
    for (const auto &pose : polygon_vertices) {
        x_min = std::min(x_min, pose.x);
        x_max = std::max(x_max, pose.x);
        y_min = std::min(y_min, pose.y);
        y_max = std::max(y_max, pose.y);
    }

    // Create rectangle corners for scanning
    std::vector<Vector3D> rectangle_poses = {
        {x_min, y_min, scan_height},
        {x_min, y_max, scan_height},
        {x_max, y_max, scan_height},
        {x_max, y_min, scan_height}
    };

    // Find closest starting pose to the current GPS position
    double min_distance = std::numeric_limits<double>::max();
    int starting_pose_index = -1;
    auto current_pos = position_listener_->get_recent_gps_msg();

    for (size_t i = 0; i < rectangle_poses.size(); ++i) {
        double dist = gps_converter_->geodeticDistance(
            current_pos->latitude_deg, current_pos->longitude_deg,
            rectangle_poses[i].getX(), rectangle_poses[i].getY()
        );
        if (dist < min_distance) {
            min_distance = dist;
            starting_pose_index = static_cast<int>(i);
        }
    }

    if (starting_pose_index == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error: No valid starting waypoint found.");
        return {};
    }

    // Determine scanning direction (longer edge first)
    int lower_index = (starting_pose_index - 1 + rectangle_poses.size()) % rectangle_poses.size();
    int higher_index = (starting_pose_index + 1) % rectangle_poses.size();

    Vector3D start_position = rectangle_poses[starting_pose_index];
    Vector3D direction_low = rectangle_poses[lower_index] - start_position;
    Vector3D direction_high = rectangle_poses[higher_index] - start_position;

    Vector3D scan_direction = (direction_low.magnitude() > direction_high.magnitude()) ? direction_low : direction_high;
    Vector3D perpendicular_direction = (scan_direction == direction_low) ? direction_high : direction_low;

    // Initialize scan waypoints
    std::vector<Vector3D> scanWaypoints = {start_position, start_position + scan_direction};

    // Compute the number of scanning sweeps required
    double total_width = gps_converter_->geodeticDistance(
        start_position.getX(), start_position.getY(),
        (start_position + perpendicular_direction).getX(),
        (start_position + perpendicular_direction).getY()
    );

    int num_sweeps = std::ceil((total_width - (sensor_width / 2.0)) / sensor_width) + 1;

    // Compute offsets for moving perpendicular to scanning direction
    double offset_x = (std::abs(scan_direction.getX()) > std::abs(scan_direction.getY())) ? sensor_width : 0.0;
    double offset_y = (std::abs(scan_direction.getX()) > std::abs(scan_direction.getY())) ? 0.0 : sensor_width;

    // Generate sweeping waypoints
    for (int i = 0; i < num_sweeps - 1; ++i) {
        Vector3D next_start(scanWaypoints.back().getX(), scanWaypoints.back().getY(), scan_height);

        // Adjust position using geodetic calculations
        next_start.setX(next_start.getX() + (180.0 / M_PI) * (offset_y / 6378137.0));
        next_start.setY(next_start.getY() + (180.0 / M_PI) * (offset_x / 6378137.0) / cos((M_PI * next_start.getX()) / 180.0));

        Vector3D next_end = (i % 2 == 0) ? next_start - scan_direction : next_start + scan_direction;
        next_end.setZ(scan_height);

        scanWaypoints.emplace_back(next_start);
        scanWaypoints.emplace_back(next_end);
    }

    return scanWaypoints;
}