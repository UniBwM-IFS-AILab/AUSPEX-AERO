#ifndef CAMERA_CONTROLLER_SIYI_ZT30
#define CAMERA_CONTROLLER_SIYI_ZT30

#include "camera_controller_base.hpp"
#include "siyi_sdk/siyi_sdk.hpp"

#include <opencv2/opencv.hpp>

class CameraControllerSiyiZT30 : public CameraControllerBase
{
public:
    CameraControllerSiyiZT30(const std::string& platform_id,
                             const std::string& ip_addr, int port,
                             int stream_width_color, int stream_height_color, int stream_bitrate_color,
                             int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                             const float fps);
    ~CameraControllerSiyiZT30();
    
    bool check_if_connected() override;
    Capabilities available_capabilities() override;
    std::optional<cv::Mat> capture_color_image() override;
    std::optional<cv::Mat> capture_ir_image() override;
    bool center() override;
    bool set_angle(float azim, float elev) override;
    bool do_sweep(float pitch, float yaw_min, float yaw_max, float speed) override;
    void stop_sweep() override;
    std::optional<float> get_distance() override;
    std::optional<std::tuple<float, float, float>> get_gimbal_attitude() override;
    
private:
    const std::string stream_url_color_;
    cv::VideoCapture capture_color_;
    const std::string stream_url_ir_;
    cv::VideoCapture capture_ir_;
    SiyiSDK camera_;
    std::thread sweep_thread_;
    std::atomic<bool> sweeping_{false};
    
    void sweep_loop(float pitch, float yaw_min, float yaw_max, float speed);
};

#endif // CAMERA_CONTROLLER_BASE