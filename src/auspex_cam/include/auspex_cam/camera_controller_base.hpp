#ifndef CAMERA_CONTROLLER_BASE
#define CAMERA_CONTROLLER_BASE

// Standard library
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

// Third-party
#include <opencv2/opencv.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

// Project messages/services
#include "auspex_msgs/srv/camera_controller_start_capture.hpp"
#include "auspex_msgs/srv/camera_controller_stop_capture.hpp"
#include "auspex_msgs/srv/camera_gimbal_center.hpp"
#include "auspex_msgs/srv/camera_gimbal_set_angle.hpp"
#include "auspex_msgs/srv/camera_gimbal_start_sweep.hpp"
#include "auspex_msgs/srv/camera_gimbal_stop_sweep.hpp"

// Other
#include "stream_handler/stream_handler.hpp"

class CameraControllerBase : public rclcpp::Node
{
public:
    explicit CameraControllerBase(const std::string& platform_id, const std::string& node_name,
                                  const std::string& ip_addr, const int port,
                                  int stream_width_color, int stream_height_color, int stream_bitrate_color,
                                  int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                                  const float fps);
    void init();
    ~CameraControllerBase();

    void start_capture();
    void stop_capture();
    bool is_capturing() const;

protected:
    struct Capabilities {
        bool image_color;
        bool image_infrared;
        bool range_finder;
        bool gimbal;
    };
    
    // Hardware specific functions
    // Always implement
    virtual Capabilities available_capabilities();
    virtual bool check_if_connected();
    // Implement for color camera
    virtual std::optional<cv::Mat> capture_color_image();
    // Implement for ir camera
    virtual std::optional<cv::Mat> capture_ir_image();
    // Implement for gimbal
    virtual bool center();
    virtual bool set_angle(float azim, float elev);
    virtual bool do_sweep(float pitch, float yaw_min, float yaw_max, float speed);
    virtual void stop_sweep();
    virtual std::optional<std::tuple<float, float, float>> get_gimbal_attitude();
    // Implement for range finder
    virtual std::optional<float> get_distance();
    
    // Stream parameters
    const float fps_;
    const int stream_width_color_;
    const int stream_height_color_;
    const int stream_bitrate_color_;
    const int stream_width_ir_;
    const int stream_height_ir_;
    const int stream_bitrate_ir_;
    
private:
    // Basic config
    const std::string platform_id_;
    
    // Stream
    std::string ip_addr_;
    int port_;
    std::unique_ptr<StreamHandler> stream_color_;
    std::unique_ptr<StreamHandler> stream_ir_;
    
    // Connection lifecycle
    bool is_connected = false;
    rclcpp::TimerBase::SharedPtr connection_timer_;
    void on_connection_timer();
    
    // Capture lifecycle
    // rclcpp::Publisher<auspex_msgs::msg::TmpFramedata>::SharedPtr frame_publisher_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    std::mutex capture_mutex_;
    void capture_frame();
    void on_capture_timer();
    
    // Services
    rclcpp::Service<auspex_msgs::srv::CameraControllerStartCapture>::SharedPtr srv_start_capture_;
    void on_srv_start_capture(const std::shared_ptr<auspex_msgs::srv::CameraControllerStartCapture::Request> req, std::shared_ptr<auspex_msgs::srv::CameraControllerStartCapture::Response> res);
    rclcpp::Service<auspex_msgs::srv::CameraControllerStopCapture>::SharedPtr srv_stop_capture_;
    void on_srv_stop_capture(const std::shared_ptr<auspex_msgs::srv::CameraControllerStopCapture::Request> req, std::shared_ptr<auspex_msgs::srv::CameraControllerStopCapture::Response> res);
    rclcpp::Service<auspex_msgs::srv::CameraGimbalCenter>::SharedPtr srv_center_;
    void on_srv_center(const std::shared_ptr<auspex_msgs::srv::CameraGimbalCenter::Request> req, std::shared_ptr<auspex_msgs::srv::CameraGimbalCenter::Response> res);
    rclcpp::Service<auspex_msgs::srv::CameraGimbalSetAngle>::SharedPtr srv_set_angle_;
    void on_srv_set_angle(const std::shared_ptr<auspex_msgs::srv::CameraGimbalSetAngle::Request> req, std::shared_ptr<auspex_msgs::srv::CameraGimbalSetAngle::Response> res);
    rclcpp::Service<auspex_msgs::srv::CameraGimbalStartSweep>::SharedPtr srv_start_sweep_;
    void on_srv_start_sweep(const std::shared_ptr<auspex_msgs::srv::CameraGimbalStartSweep::Request> req, std::shared_ptr<auspex_msgs::srv::CameraGimbalStartSweep::Response> res);
    rclcpp::Service<auspex_msgs::srv::CameraGimbalStopSweep>::SharedPtr srv_stop_sweep_;
    void on_srv_stop_sweep(const std::shared_ptr<auspex_msgs::srv::CameraGimbalStopSweep::Request> req, std::shared_ptr<auspex_msgs::srv::CameraGimbalStopSweep::Response> res);
    
    // Helper functions
    std::vector<uint8_t> compress_image_to_jpg(cv::Mat image, int quality);
};

#endif // CAMERA_CONTROLLER_BASE