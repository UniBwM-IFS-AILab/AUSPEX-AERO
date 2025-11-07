#pragma once

#include "camera_controller_base.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <cv_bridge/cv_bridge.h>

#include <libcamera/framebuffer_allocator.h> 
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>
#include <libcamera/formats.h>
#include <sys/mman.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

class CameraControllerRPI5 : public CameraControllerBase
{
public:
    CameraControllerRPI5(const std::string& platform_id,
                         const std::string& ip_addr, int port,
                         int stream_width_color, int stream_height_color, int stream_bitrate_color,
                         int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                         const float fps);
    ~CameraControllerRPI5() override;

    bool check_if_connected() override;
    Capabilities available_capabilities() override;
    std::optional<cv::Mat> capture_color_image() override;

private:
    // ---- libcamera plumbing ----
    void initCamera();
    void shutdownCamera();

    void startStreaming();
    void stopStreaming();

    void onRequestCompleted_(libcamera::Request* req);
    void workerLoop_();

    cv::Mat gammaCorrect(const cv::Mat& src, double gamma);

    struct Mapped {
        void*  addr   = nullptr;
        size_t length = 0;
        int    width  = 0;
        int    height = 0;
        int    bytesPerLine = 0;
    };

    // Config requested output size (what your stream expects)
    const int out_w_;
    const int out_h_;
    const float fps_;

    // libcamera objects
    std::unique_ptr<libcamera::CameraManager> cam_mgr_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_ = nullptr;

    std::vector<libcamera::FrameBuffer*> buffers_;
    std::vector<Mapped> mapped_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    // white balance + image staging
    cv::Ptr<cv::xphoto::SimpleWB> wb_;

    // worker & job queue (latest-wins)
    std::atomic<bool> running_{false};
    std::thread worker_;
    struct RawJob { int idx = -1; uint64_t seq = 0; };
    std::mutex q_mtx_;
    std::condition_variable q_cv_;
    std::deque<RawJob> rawQ_;

    // latest processed image
    std::mutex latest_mtx_;
    cv::Mat latest_rgb_;

    // connection flag for base::check_if_connected
    std::atomic<bool> connected_{false};
};
