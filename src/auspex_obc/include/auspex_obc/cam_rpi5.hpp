#ifndef CAM_RPI5_HPP
#define CAM_RPI5_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>
#include <cv_bridge/cv_bridge.hpp>
#include <libcamera/camera_manager.h>
#include <opencv2/xphoto/white_balance.hpp>
#include <libcamera/framebuffer_allocator.h>
#include "auspex_fci/position_listener_base.hpp"
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <sys/mman.h>

using namespace std::placeholders;
using libcamera::Stream;


#include "msg_context_cpp/message_loader.hpp"

class RPI5_CameraPublisher : public rclcpp::Node{
public:
    RPI5_CameraPublisher(std::string name_prefix = "") : Node(name_prefix + "_" + "rpi5_cam_publisher") {
        image_publisher_ = this->create_publisher<DroneImage>(name_prefix +"/raw_camera_stream", 10);
        _name_prefix = name_prefix;
        transmitSize_width = 320;
        transmitSize_height = 320;

        initCamera();

		fps_ = 4.0;
		double tpu = 1.0/fps_;

        // Timer to trigger image capture
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(1000*tpu)),
            std::bind(&RPI5_CameraPublisher::processRequest, this));
        timer_->cancel(); 
    }

    ~RPI5_CameraPublisher(){
        if (camera_){
            camera_->stop();
        }

        if (camera_){
            camera_->release();
        }

        if (cameraManager_){
            cameraManager_->stop();
        }

        RCLCPP_INFO(this->get_logger(), "Shutting down camera...");
    }

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
		}
	}

	void take_single_image(){
        processRequest();
	}

    bool isCameraRunning(){
        return timer_running_;
    }
private:
    void initCamera(){
        cameraManager_ = std::make_unique<libcamera::CameraManager>();
        wb_ = cv::xphoto::createSimpleWB();

        if (cameraManager_->start() != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start Camera Manager.");
            throw std::runtime_error("Camera Manager start failed");
        }

        if (cameraManager_->cameras().empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No cameras found.");
            throw std::runtime_error("No cameras found.");
        }

        camera_ = cameraManager_->cameras()[0];
        if (!camera_ || camera_->acquire())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera.");
            throw std::runtime_error("Camera acquisition failed.");
        }

        config_ = camera_->generateConfiguration({libcamera::StreamRole::Raw});
        config_->at(0).size = {2028,1080};//;{2028, 1520};
        config_->at(0).pixelFormat = libcamera::formats::SBGGR12; //12
        config_->at(0).stride = 2048;//2048;

        if (config_->validate() == libcamera::CameraConfiguration::Invalid)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid camera configuration.");
            throw std::runtime_error("Invalid camera configuration.");
        }

        if (camera_->configure(config_.get()))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure camera.");
            throw std::runtime_error("Camera configuration failed.");
        }

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(config_->at(0).stream()) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffers.");
            throw std::runtime_error("Buffer allocation failed.");
        }

        auto &buffers = allocator_->buffers(config_->at(0).stream());
        if (buffers.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No buffers allocated.");
            throw std::runtime_error("No buffers allocated.");
        }

        buffer_ = buffers[0].get(); // Use a single buffer for simplicity

        //camera_->requestCompleted.connect(camera_.get(), std::bind(&RPI5_CameraPublisher::processRequest, this, _1));

        request_ = camera_->createRequest();
        if (!request_){
            RCLCPP_ERROR(this->get_logger(), "Failed to create request.");
            return;   
        }
        if (!buffer_) {
            RCLCPP_ERROR(this->get_logger(), "Buffer is null.");
            return;
        }

        if (request_->addBuffer(config_->at(0).stream(), buffer_) < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to attach buffer to request.");
            return;
        }


        if (camera_->start())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start the camera.");
            throw std::runtime_error("Failed to start the camera.");
        }

        RCLCPP_INFO(this->get_logger(), "Camera initialized successfully.");
    }

    void processRequest(){   
        

        if (camera_->queueRequest(request_.get()) < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to queue the request.");
            return;
        }

        //RCLCPP_INFO(this->get_logger(), "Got an image. Publishing...");

        while(request_->status() != libcamera::Request::RequestComplete){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if(request_->status() ==libcamera::Request::RequestCancelled){
                RCLCPP_WARN(this->get_logger(), "Request was cancelled.");
                return;
            }
        }

        // Retrieve buffer data
        libcamera::FrameBuffer *buffer = request_->buffers().begin()->second;
        if (!buffer) {
            RCLCPP_ERROR(this->get_logger(), "Request contains no buffers.");
            return;
        }
        const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];

        void *data = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), plane.offset);
        if (data == MAP_FAILED){
            RCLCPP_ERROR(this->get_logger(), "Failed to mmap buffer.");
            return;
        }

        uint8_t *rawData = static_cast<uint8_t *>(data);

        cv::Mat bayerImage(config_->at(0).size.height, config_->at(0).size.width, CV_16UC1);
        for (uint y = 0; y < config_->at(0).size.height; ++y)
        {
            memcpy(bayerImage.ptr(y), rawData + y * config_->at(0).stride, config_->at(0).size.width * 2);
        }

        /*
        DO STUFF WITH IMAGE
        */


        // convert to 8 bit channel
        bayerImage.convertTo(bayerImage, CV_8UC1, 1.0 / 256.0);
        cv::cvtColor(bayerImage, bayerImage, cv::COLOR_BayerBG2RGB);
        cv::resize(bayerImage, bayerImage, cv::Size(transmitSize_width, transmitSize_height), 0, 0, cv::INTER_LINEAR);

        // std::vector<cv::Mat> channels(3);
        // cv::split(bayerImage, channels);
        // channels[2] *= 2.0;   // Red channel
        // channels[1] *= 1.0; // Green channel
        // channels[0] *= 2.0;  // Blue channel
        // cv::merge(channels, bayerImage);
        // cv::threshold(bayerImage, bayerImage, 255, 255, cv::THRESH_TRUNC);

        wb_->balanceWhite(bayerImage, bayerImage);
  
        /*
        Convert Image to Message and Send
        */
        auto image_msg = DroneImage();
        auto compressed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bayerImage).toCompressedImageMsg();
        msg.platform_id = _name_prefix;      // drone ID
        msg.team_id = "drone_team";  

        if(gps_listener_ != nullptr){
			msg.gps_position.latitude = gps_listener_->get_recent_gps_msg()->lat;
			msg.gps_position.longitude = gps_listener_->get_recent_gps_msg()->lon;
			msg.gps_position.altitude = gps_listener_->get_recent_gps_msg()->alt;
		}

        image_msg.image_compressed = *compressed_image_msg;
        image_msg.image_compressed.format = "png";
        image_msg.fps = int(fps_);
        image_msg.res_width = transmitSize_width;
        image_msg.res_height = transmitSize_height;

        this->image_publisher_->publish(image_msg);


        //Unmap buffer etc
        munmap(data, plane.length);
        request_->reuse();
        if (request_->addBuffer(config_->at(0).stream(), buffer_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reattach buffer to the request.");
            return;
        }
    }


private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<DroneImage>::SharedPtr image_publisher_;
	std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener_;

    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::unique_ptr<libcamera::Request> request_;
    libcamera::FrameBuffer *buffer_;
    cv::Ptr<cv::xphoto::SimpleWB> wb_;

    std::string _name_prefix;

    int transmitSize_width;
    int transmitSize_height;

    bool timer_running_ = false;
    int vhcl_id = -1;
	int fps_ = 0;
};

#endif