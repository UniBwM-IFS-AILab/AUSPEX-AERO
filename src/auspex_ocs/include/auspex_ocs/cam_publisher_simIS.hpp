#ifndef SIM_IS_CAM_PUBLISHER_HPP
#define SIM_IS_CAM_PUBLISHER_HPP

#include "cam_publisher_base.hpp"
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <zmq.hpp>
#include <nlohmann/json.hpp>

/**
 * @brief Camera publisher for Isaac Sim simulator via ZMQ
 * 
 */
class SimCamPublisherIS : public CamPublisherBase {
public:
    SimCamPublisherIS(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps);
    ~SimCamPublisherIS() override;

    void captureFrame() override;

private:
    int vhcl_id_;
    int zmq_port_;
    std::string camera_name_;
    
    // ZMQ components
    zmq::context_t zmq_context_;
    zmq::socket_t zmq_socket_;
    
    // Threading for ZMQ reception
    std::thread zmq_thread_;
    std::atomic<bool> zmq_running_;
    
    // Frame storage with thread safety
    cv::Mat latest_frame_;
    mutable std::mutex frame_mutex_;
    std::atomic<bool> frame_available_;
    
    // ZMQ receiver loop
    void zmq_receiver_loop();
    bool receive_zmq_frame(cv::Mat& frame);
    
    mutable std::mutex capture_mutex_;
};

#endif // SIM_CAM_PUBLISHER_HPP


