#include "auspex_obc/obc_node.h"
#include "auspex_obc/ext_data_listener.hpp"
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

std::string get_values_from_config(json& config, float* return_img_width, float* return_img_height, float* return_cam_fps, std::string* payload) {

	std::string platform_id = config["platform_id"];
	*return_img_width = config["sensors"][0]["specifications"]["image_size"]["width"];
	*return_img_height = config["sensors"][0]["specifications"]["image_size"]["height"];
	*return_cam_fps = config["sensors"][0]["specifications"]["image_fps"];
	*payload = config["payload"][0]["type"];

	return platform_id;
}

/**
* @brief Get the IP address of tap0 interface if it exists and starts with 10.8.
* @return IP address as string, or "0.0.0.0" if tap0 not found or doesn't match criteria
*/
std::string getTAP0IpAddress() {
	struct ifaddrs *ifaddr, *ifa;
	char ip_str[INET_ADDRSTRLEN];
	
	// Get list of network interfaces
	if (getifaddrs(&ifaddr) == -1) {
		return "0.0.0.0";
	}
	
	// Iterate through interfaces
	for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
		if (ifa->ifa_addr == nullptr) continue;
		// Check if this is tap0 interface and is IPv4
		if (strcmp(ifa->ifa_name, "tap0") == 0 && ifa->ifa_addr->sa_family == AF_INET) {
			struct sockaddr_in* sin = (struct sockaddr_in*)ifa->ifa_addr;
			
			// Convert IP to string
			if (inet_ntop(AF_INET, &sin->sin_addr, ip_str, INET_ADDRSTRLEN) != nullptr) {
				std::string ip_address(ip_str);
				// Check if IP starts with "10.8."
				if (ip_address.substr(0, 5) == "10.8.") {
					freeifaddrs(ifaddr);
					return ip_address;
				}
			}
		}
	}
	
	freeifaddrs(ifaddr);
	return "0.0.0.0";
}

// for running multiple nodes, see as Example https://docs.ros.org/en/foxy/Tutorials/Demos/Intra-Process-Communication.html
int main(int argc, char* argv[]) {
	RCLCPP_INFO(rclcpp::get_logger("main"),"Starting nodes...");
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	std::string file_path = "";

	// Important to ba an env variable because omits airsim while building
	std::string FC_TYPE = getEnvVar("FC_TYPE");
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

	// RTSP vars
	std::string platform_ip = getTAP0IpAddress(); // to be able to publish sim stream over VPN
	int port_offset = 0; // should be zero for real platforms

	// If in simulation mode use command line argument as names
	if(FC_TYPE.find("SIMULATED") != std::string::npos){
		if(argv[1]!=NULL && strcmp(argv[1],"--ros-args")!=0){
			
			config["platform_id"] = argv[1];
			port_offset = std::stoi(std::string(argv[1]).substr(std::string(argv[1]).rfind('_')+1));

		}else{
			config["platform_id"] = "vhcl_init_failed";
		}
	}else{
		if(platform_ip == "0.0.0.0") {
			RCLCPP_INFO(rclcpp::get_logger("main"), "No tap0 interface found, shutting down...");
			return -1;
		}
	}

	float cam_fps_ = 1.0;
	float image_height = 1.0;
	float image_width = 1.0;
	std::string payload = "";

	// Try to register platform to BK
	std::string platform_id = get_values_from_config(config, &image_width, &image_height, &cam_fps_, &payload);

	if(platform_id == ""){
		RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
		rclcpp::shutdown();
		return -1;
	}

	std::shared_ptr<FC_Interface_Base> fc_interface;
	RCLCPP_INFO(rclcpp::get_logger("main"), "Platform: %s launching ...", platform_id.c_str());

	if(FC_TYPE.find("ANAFI") != std::string::npos){
		std::shared_ptr<VehicleStatusListener_Base> vehicle_status_listener = std::make_shared<VehicleStatusListener_ANAFI>(platform_id);
		std::shared_ptr<VehicleGlobalPositionListener_Base> position_listener = std::make_shared<VehicleGlobalPositionListener_ANAFI>(platform_id);
		fc_interface = std::make_shared<FC_Interface_ANAFI>(vehicle_status_listener, position_listener, platform_id);
	}else{
		fc_interface = std::make_shared<FC_Interface_MAVSDK>(platform_id, FC_TYPE);
	}

	if(fc_interface->get_is_initialized() == false){
		RCLCPP_ERROR(rclcpp::get_logger("main"), "FC Interface %s could not be initialized, shutting down..", platform_id.c_str());
		rclcpp::shutdown();
		return -1;
	}

	// auto cam_publisher = std::make_shared<CamPublisher>(platform_id, platform_ip, port_offset, image_height, image_width, cam_fps_);
	int bitrate = 1000; // in kbit/s
	#ifdef SWITCH_CAM_HEADER
		#if SWITCH_CAM_HEADER == 0
			auto cam_controller = std::make_shared<CameraController>(platform_id, "camera_controller_empty", platform_ip, 8554+port_offset, image_width, image_height, bitrate, image_width, image_height, bitrate, cam_fps_);
		#else
			auto cam_controller = std::make_shared<CameraController>(platform_id, platform_ip, 8554+port_offset, image_width, image_height, bitrate, image_width, image_height, bitrate, cam_fps_);
			cam_controller->init();
		#endif
	#endif

	auto platform_state_publisher = std::make_shared<PlatformStatePublisher>(platform_id, platform_ip, config);

	RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for GPS init...");
	while (!fc_interface->get_position_listener()->get_first_gps_future()) {
		RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for GPS init...");
		fc_interface->get_position_listener()->is_home_position_set();
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	RCLCPP_INFO(rclcpp::get_logger("main"), "Launching Offboard Control node...");
	auto external_data_listener = std::make_shared<ExternalDataListener>(platform_id);
	auto controller = std::make_shared<OffboardController>(fc_interface->get_vehicle_status_listener(), fc_interface->get_position_listener(), fc_interface, platform_state_publisher, cam_controller, external_data_listener, platform_id, payload);


	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(fc_interface->get_vehicle_status_listener());
	executor.add_node(fc_interface->get_position_listener());
	executor.add_node(cam_controller);
	executor.add_node(platform_state_publisher);
	executor.add_node(external_data_listener);
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
