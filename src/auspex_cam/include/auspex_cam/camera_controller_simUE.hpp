#pragma once

#include "camera_controller_base.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

class CameraControllerSimUE : public CameraControllerBase
{
public:
    CameraControllerSimUE(const std::string& platform_id,
                          const std::string& ip_addr, int port,
                          int stream_width_color, int stream_height_color, int stream_bitrate_color,
                          int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                          const float fps);
    ~CameraControllerSimUE();

    bool check_if_connected() override;
    Capabilities available_capabilities() override;
    std::optional<cv::Mat> capture_color_image() override;

private:
    int vhcl_id_;
    mutable std::mutex capture_mutex_;
};
