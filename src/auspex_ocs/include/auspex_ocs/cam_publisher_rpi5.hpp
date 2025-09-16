#ifndef RPI5_CAM_PUBLISHER_HPP
#define RPI5_CAM_PUBLISHER_HPP

#include "cam_publisher_base.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/xphoto/white_balance.hpp>
#include <chrono>
#include <thread>
#include <sys/mman.h>

/**
 * @brief Camera publisher for Raspberry Pi 5 using libcamera.
 */
class RPI5CamPublisher : public CamPublisherBase {
public:
    RPI5CamPublisher(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps);
    ~RPI5CamPublisher() override;

    void captureFrame() override;

private:

    void initCamera();
    std::unique_ptr<libcamera::CameraManager>              cameraManager_;
    std::shared_ptr<libcamera::Camera>                     camera_;
    std::unique_ptr<libcamera::CameraConfiguration>        config_;
    std::unique_ptr<libcamera::FrameBufferAllocator>       allocator_;
    std::unique_ptr<libcamera::Request>                    request_;
    libcamera::FrameBuffer*                                buffer_;
    cv::Ptr<cv::xphoto::SimpleWB>                          wb_;
};

#endif // RPI5_CAM_PUBLISHER_HPP