#pragma once

#include "camera_controller_base.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// NEW: ZMQ / JSON
#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <atomic>

class CameraControllerSimIS : public CameraControllerBase
{
public:
    CameraControllerSimIS(const std::string& platform_id,
                          const std::string& ip_addr, int port,
                          int stream_width_color, int stream_height_color, int stream_bitrate_color,
                          int stream_width_ir, int stream_height_ir, int stream_bitrate_ir,
                          const float fps);
    ~CameraControllerSimIS();

    bool check_if_connected() override;
    Capabilities available_capabilities() override;
    std::optional<cv::Mat> capture_color_image() override;

private:
    // Unused in this ZMQ sim path, but kept to match your header
    cv::VideoCapture capture_color_;

    // ---- Ported-from-old members ----
    int vhcl_id_{-1};
    int zmq_port_{-1};
    std::string camera_name_;

    zmq::context_t zmq_context_{1};
    zmq::socket_t zmq_socket_{zmq_context_, ZMQ_SUB};
    std::thread zmq_thread_;
    std::atomic<bool> zmq_running_{false};

    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    std::atomic<bool> frame_available_{false};

    // For lightweight health reporting
    std::atomic<uint64_t> frames_received_{0};

    // ---- Helpers ----
    void initCamera();
    void shutdownCamera();
    void zmq_receiver_loop();
    bool receive_zmq_frame(cv::Mat& frame);
};
