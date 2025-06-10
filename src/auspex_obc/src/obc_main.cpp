#include "auspex_obc/obc_node.h"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cstdlib>  // for setenv

using json = nlohmann::json;

std::string getEnvVar( std::string const & key ){
    char * val = getenv( key.c_str() );
    return val == NULL ? std::string("") : std::string(val);
}

std::string getHomeDirectory() {
    const char* home = std::getenv("HOME");
    if (home == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "HOME environment variable is not set.");
        return "";
    }
    return std::string(home);
}

std::string register_platform(json& config){
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("platform_registry_client_node");
	rclcpp::Client<ExistsKnowledge>::SharedPtr exists_knowledge_client_ = node->create_client<ExistsKnowledge>("exists_knowledge");
	rclcpp::Client<InsertKnowledge>::SharedPtr insert_knowledge_client_ = node->create_client<InsertKnowledge>("insert_knowledge");
	auto publisher_ = node->create_publisher<PlatformCapabilities>("platform_capabilities", 10);

	while (!exists_knowledge_client_->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return "";
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "exists_knowledge_client_ service not available, waiting again...");
	}

	std::string platform_id = config["platform_id"];

	auto exists_request = std::make_shared<ExistsKnowledge::Request>();
	exists_request->collection = "platform";
	exists_request->path = "$[?(@.platform_id==\"" + platform_id + "\")]";
	// Asynchronously call the service
	auto exists_future_result  = exists_knowledge_client_->async_send_request(exists_request);
	// Block until the result is available or timeout occurs
    auto spin_result = rclcpp::spin_until_future_complete(node, exists_future_result);
	if (spin_result != rclcpp::FutureReturnCode::SUCCESS) {
		if (spin_result == rclcpp::FutureReturnCode::TIMEOUT) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call to ExistsKnowledge %s timed out.", platform_id.c_str());
		} else if (spin_result == rclcpp::FutureReturnCode::INTERRUPTED) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call to ExistsKnowledge %s was interrupted.", platform_id.c_str());
		} else {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call to ExistsKnowledge %s failed with unknown error.", platform_id.c_str());
		}
		return "";
	}

	auto exists_result = exists_future_result.get();
	if (exists_result->exists) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Platform %s already existing.", platform_id.c_str());
		return "";
	}

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Platform %s does not exist in Knowledgebase, now inserting...", platform_id.c_str());

	while (!insert_knowledge_client_->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return "";
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "insert_knowledge_client_ service not available, waiting again...");
	}

	auto insert_request = std::make_shared<InsertKnowledge::Request>();
	insert_request->collection = "platform";
	insert_request->path = "$";
	insert_request->entity = "{\"platform_id\":\"" + platform_id + "\"}";
	// Asynchronously call the service
	auto insert_future_result  = insert_knowledge_client_->async_send_request(insert_request);

	if (!(rclcpp::spin_until_future_complete(node, insert_future_result) == rclcpp::FutureReturnCode::SUCCESS)){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call to InsertKnowledge %s timed out.", platform_id.c_str());
		return "";
	}

	auto insert_result = insert_future_result.get();
	if (insert_result->success) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Platform %s registered.", platform_id.c_str());
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Platform %s could not be registered.", platform_id.c_str());
		return "";
	}

	std::vector<SensorCapabilities> sensor_capabilities_msg;

	for(auto &sensor : config["sensors"]){
		auto sensor_msg = SensorCapabilities();
		sensor_msg.sensor_id = sensor["id"];

		auto sensor_mode = SensorMode();
		if(sensor["type"] == "eo_camera"){
			sensor_mode.value = SensorMode::SENSOR_MODE_EO;
		}else if(sensor["type"] == "ir_camera") {
			sensor_mode.value = SensorMode::SENSOR_MODE_IR;
		}
		sensor_msg.sensor_mode = sensor_mode;

		sensor_msg.fov_hor_min = sensor["specifications"]["fov"]["horizontal"]["min"];
		sensor_msg.fov_hor_max = sensor["specifications"]["fov"]["horizontal"]["max"];
		sensor_msg.fov_vert_min = sensor["specifications"]["fov"]["vertical"]["min"];
		sensor_msg.fov_vert_max = sensor["specifications"]["fov"]["vertical"]["max"];
		sensor_msg.image_width = sensor["specifications"]["image_width"];
		sensor_msg.image_height = sensor["specifications"]["image_height"];

		sensor_capabilities_msg.push_back(sensor_msg);
	}

	auto platform_capabilities_msg = PlatformCapabilities();
    platform_capabilities_msg.platform_id = platform_id;
    platform_capabilities_msg.model_info = config["platform_details"]["model"];

    platform_capabilities_msg.max_flight_duration = config["platform_details"]["max_flight_duration"];
    platform_capabilities_msg.max_flight_height = config["platform_details"]["max_flight_height"];
    platform_capabilities_msg.max_velocity = config["platform_details"]["max_velocity"];
    platform_capabilities_msg.turning_radius = config["platform_details"]["turning_radius"];

	auto platform_class = PlatformClass();
	platform_class.value = PlatformClass::PLATFORM_CLASS_DRONE;
    platform_capabilities_msg.platform_class = platform_class;
    platform_capabilities_msg.sensor_caps = sensor_capabilities_msg;


	publisher_->publish(platform_capabilities_msg);
	publisher_->publish(platform_capabilities_msg);

	return platform_id;
}


// for running multiple nodes, see as Example https://docs.ros.org/en/foxy/Tutorials/Demos/Intra-Process-Communication.html
int main(int argc, char* argv[]) {
	RCLCPP_INFO(rclcpp::get_logger("main"),"Starting nodes...");
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	std::string file_path = "";

	// Important to ba an env variable because omits airsim while building
	std::string FC_TYPE = getEnvVar("FC_TYPE");
	std::string OBC_TYPE = getEnvVar("OBC_TYPE"); // Not used. Just for completeness
	std::string CAM_TYPE = getEnvVar("CAM_TYPE");

	file_path =  getHomeDirectory() + "/auspex_params/platform_properties/platform_properties.json";

	// Read in Json file
	std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Could not open the file " << file_path << std::endl;
        return -1;
    }

	// Parse the JSON content
    json config;
	try{
    	file >> config;
	}catch (const nlohmann::json::parse_error& e) {
        std::cerr << "JSON Parse Error: " << e.what() << std::endl;
    }
    catch (const std::runtime_error& e) {
        std::cerr << "Runtime Error: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

	// If in simulation mode use command line argument as names
	if(FC_TYPE.find("SIMULATED") != std::string::npos){
		if(argv[1]!=NULL && strcmp(argv[1],"--ros-args")!=0){
			config["platform_id"] = argv[1];
		}else{
			config["platform_id"] = "vhcl_init_failed";
		}
	}

	// Try to register platform to BK
	std::string platform_id = register_platform(config);

	if(platform_id == ""){
		RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
		rclcpp::shutdown();
		return -1;
	}

	std::shared_ptr<VehicleStatusListener_Base> vehicle_listener;
	std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener;
	std::shared_ptr<FC_Interface_Base> fc_interface;

	RCLCPP_INFO(rclcpp::get_logger("main"), "Platform: %s launching ...", platform_id.c_str());

	if(FC_TYPE.find("PX4") != std::string::npos){
		vehicle_listener = std::make_shared<VehicleStatusListener_PX4>(platform_id);
		position_listener = std::make_shared<VehicleGlobalPositionListener_PX4>(platform_id);
		fc_interface = std::make_shared<FC_Interface_PX4>(vehicle_listener, position_listener, platform_id);
	}else if(FC_TYPE.find("ANAFI") != std::string::npos){
		vehicle_listener = std::make_shared<VehicleStatusListener_ANAFI>(platform_id);
		position_listener = std::make_shared<VehicleGlobalPositionListener_ANAFI>(platform_id);
		fc_interface = std::make_shared<FC_Interface_ANAFI>(vehicle_listener, position_listener, platform_id);
	}

	auto cam_publisher = std::make_shared<CamPublisher>(platform_id, 1.0);
	auto drone_state_publisher = std::make_shared<DroneStatePublisher>(platform_id, config);

	RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for GPS Position Publisher...");
	rclcpp::spin_until_future_complete(position_listener, position_listener->get_next_gps_future());

	RCLCPP_INFO(rclcpp::get_logger("main"), "Launching Offboard Control node...");
	auto controller = std::make_shared<OffboardController>(vehicle_listener, position_listener, fc_interface, drone_state_publisher, cam_publisher, platform_id);


	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(vehicle_listener);
	executor.add_node(position_listener);
	executor.add_node(cam_publisher);
	executor.add_node(drone_state_publisher);
	executor.add_node(controller);

	RCLCPP_INFO(rclcpp::get_logger("main"), "Initialized offboard controller. Now spinning...");

    try{
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception caught while spinning: %s", e.what());
    }

	RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
	rclcpp::shutdown();
	return 0;
}
