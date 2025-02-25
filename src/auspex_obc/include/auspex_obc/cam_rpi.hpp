#ifndef CAM_RPI_HPP
#define CAM_RPI_HPP

#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include "auspex_fci/position_listener_base.hpp"


#include "msg_context_cpp/message_loader.hpp"

class RPI_CameraPublisher : public rclcpp::Node {
public:
    RPI_CameraPublisher(std::string name_prefix = "") : Node(name_prefix + "_" + "rpi_cam_publisher") {
        _publisher = this->create_publisher<DroneImage>(name_prefix +"/raw_camera_stream", 10);
        _name_prefix = name_prefix;
        
        // Set camera parameters
         if(!debug){
            _cap = cv::VideoCapture(0);
            _cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280); //640
            _cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720); //360

            if (!_cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
                rclcpp::shutdown();
            }
        }

        double fps = 30.0;
        _fps = fps;
		double tpu = 1.0/fps;

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(int(tpu*1000)), 
            std::bind(&RPI_CameraPublisher::timer_callback, this)
        );
        _timer->cancel();  // Start with the timer stopped
    }

    void timer_callback() {
        cv::Mat frame;
        if(!debug){
            if (_cap.read(frame)) {
                auto image_msg = DroneImage();
                // Convert to ROS image message
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg();
                msg.platform_id = _name_prefix;      // drone ID
                msg.team_id = "drone_team";  
                image_msg.image_compressed = *msg;
                image_msg.image_compressed.format = "png";  // Corrected way to set the format
                image_msg.fps = int(this->_fps);
                image_msg.res_width = 1280;
                image_msg.res_height = 720;

                if(gps_listener_ != nullptr){
                    image_msg.gps_position.latitude = gps_listener_->get_recent_gps_msg()->lat;
                    image_msg.gps_position.longitude = gps_listener_->get_recent_gps_msg()->lon;
                    image_msg.gps_position.altitude = gps_listener_->get_recent_gps_msg()->alt;
                }
                this->_publisher->publish(image_msg);
                //RCLCPP_INFO(this->get_logger(), "Publishing");
            }else{
                RCLCPP_INFO(this->get_logger(), "READ FRAME ERROR");
            }
        }
    }
    
	void start_Camera(std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener){
		if (!timer_running_) {
			this->gps_listener_ = gps_listener;
			_timer->reset();  // Start the timer
			timer_running_ = true;
			RCLCPP_INFO(this->get_logger(), "Camera started.");
		}
	}
	void stop_Camera(){
		if (timer_running_) {
			_timer->cancel();  // Stop the timer
			timer_running_ = false;
			RCLCPP_INFO(this->get_logger(), "Camera stopped.");
		}
	}

	void take_single_image(){
		cv::Mat frame;
        if (_cap.read(frame)) {
             auto image_msg = DroneImage();
            // Convert to ROS image message
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg();
            image_msg.image_compressed = *msg;
            image_msg.image_compressed.format = "png";  // Corrected way to set the format
            _publisher->publish(image_msg);
        }else{
            RCLCPP_INFO(this->get_logger(), "READ FRAME ERROR");
        }
	}

    ~RPI_CameraPublisher() {
        _cap.release();
    }

    bool isCameraRunning(){
        return timer_running_;
    }

private:
    cv::VideoCapture _cap;
    rclcpp::Publisher<DroneImage>::SharedPtr _publisher;
	std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener_;
    rclcpp::TimerBase::SharedPtr _timer;	
    bool timer_running_ = false;
    bool debug = true;
    std::string _name_prefix;
    int _fps = 0;

};


#endif