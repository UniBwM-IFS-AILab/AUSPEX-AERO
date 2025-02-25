#ifndef CAM_SIM_HPP
#define CAM_SIM_HPP

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "auspex_fci/position_listener_base.hpp"

#include "msg_context_cpp/message_loader.hpp"

class SimCamPublisher : public rclcpp::Node{
  public:
    SimCamPublisher(std::string name_prefix = ""): Node(name_prefix + "_" + "sim_cam_publisher") {
		vhcl_id = stoi(name_prefix.substr(4, name_prefix.size() - 2));
		const char* ip_addr = std::getenv("WSL_HOST_IP"); 
		sim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::MultirotorRpcLibClient(ip_addr));
		
		_name_prefix = name_prefix;
		image_publisher_ = this->create_publisher<DroneImage>(name_prefix +"/raw_camera_stream", 10); 

		double fps = 4.0;
		_fps = fps;
		double tpu = 1.0/fps;

		timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*tpu)), std::bind(&SimCamPublisher::timer_callback, this));
		timer_->cancel();  // Start with the timer stopped
	};
    
	void timer_callback(){
		auto msg = DroneImage();
		//Can be changed to rgb
		//enum class ImageType : int{ Scene = 0,DepthPlanar,DepthPerspective,DepthVis,DisparityNormalized,Segmentation,SurfaceNormals,Infrared,OpticalFlow,OpticalFlowVis,Count //must be last};
		
		//std::vector<uint8_t> raw_image = sim_client_->simGetImage("front_30", msr::airlib::ImageCaptureBase::ImageType::Scene, "Drone"+std::to_string(vhcl_id+1));

		std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests = {
            msr::airlib::ImageCaptureBase::ImageRequest("front_30", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)
        };

		std::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses = sim_client_->simGetImages(requests,"Drone"+std::to_string(vhcl_id+1) );
		std::vector<uint8_t>& raw_image = responses[0].image_data_uint8;
		
		
		if (raw_image.empty()) {
			std::cerr << "Failed to capture image!" << std::endl;
			return;
		}
		int width = responses[0].width;
		int height = responses[0].height;

		cv::Mat image(height, width, CV_8UC3, const_cast<uint8_t*>(raw_image.data()));
		cv::Mat rgb_image;
		cv::cvtColor(image, rgb_image, cv::COLOR_RGB2RGBA);

		std::vector<uint8_t> png_image;
    	bool success = cv::imencode(".png", rgb_image, png_image);

		msg.platform_id = _name_prefix;      // drone ID
        msg.team_id = "drone_team";  
		msg.image_compressed.data = png_image;
		msg.image_compressed.header.stamp = this->get_clock()->now();
		msg.image_compressed.format = "png" ;
		msg.fps = int(this->_fps);
		msg.res_width = width;
		msg.res_height = height;
		if(gps_listener_ != nullptr){
			msg.gps_position.latitude = gps_listener_->get_recent_gps_msg()->lat;
			msg.gps_position.longitude = gps_listener_->get_recent_gps_msg()->lon;
			msg.gps_position.altitude = gps_listener_->get_recent_gps_msg()->alt;
		}
	
		image_publisher_->publish(msg);
	};


	void start_Camera(std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener){
		if (!timer_running_) {
			this->gps_listener_ = gps_listener;
			timer_->reset();  // Start the timer
			timer_running_ = true;
			RCLCPP_INFO(this->get_logger(), "Camera started.");
		}
	}

	void stop_Camera(){
		if (timer_running_) {
			timer_->cancel();  // Start the timer
			timer_running_ = false;
			RCLCPP_INFO(this->get_logger(), "Camera stopped.");
			
			auto msg = DroneImage();
			msg.image_compressed.format = "empty" ;
			image_publisher_->publish(msg);
		}
	}

	void take_single_image(){
		auto msg = DroneImage();
		std::vector<uint8_t> png_image = sim_client_->simGetImage("front_30", msr::airlib::ImageCaptureBase::ImageType::Scene, "Drone"+std::to_string(vhcl_id+1));

		msg.image_compressed.data = png_image;
		msg.image_compressed.header.stamp = this->get_clock()->now();
		msg.image_compressed.format = "png" ;
		
		image_publisher_->publish(msg);
	}

	bool isCameraRunning(){
        return timer_running_;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<DroneImage>::SharedPtr image_publisher_;
	std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener_;
	std::string _name_prefix;

	bool timer_running_ = false;

	std::unique_ptr<msr::airlib::RpcLibClientBase> sim_client_;

	int vhcl_id = -1;

	int _fps = 0;

};


#endif



