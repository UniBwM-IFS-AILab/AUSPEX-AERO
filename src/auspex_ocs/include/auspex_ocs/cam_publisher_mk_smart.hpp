#ifndef MK_SMART_CAM_PUBLISHER_HPP
#define MK_SMART_CAM_PUBLISHER_HPP

#include "cam_publisher_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class MkSmartCamPublisher : public CamPublisherBase{
public:
    MkSmartCamPublisher(std::string platform_id, const float fps);
    ~MkSmartCamPublisher() override;

    void captureFrame() override;

private:
    const std::string stream_url_;
    cv::VideoCapture capture_;
};

#endif