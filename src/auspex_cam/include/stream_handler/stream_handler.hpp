// stream_handler.hpp
#pragma once

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>
#include <string>

#include "stream_handler/rtsp_server_manager.hpp"

class StreamHandler {
public:
    StreamHandler(const std::string &bind_addr, int port,
                  const std::string &mount, int width, int height,
                  int fps, int bitrate_kbps);
    ~StreamHandler();

    void start();
    void stop();
    bool submit_frame(const cv::Mat &frame);
    
    int get_port() const { return port_; }
    std::string get_ip() const { return bind_addr_; }
    std::string get_mount() const { return mount_; }

private:
    static void on_media_configure(GstRTSPMediaFactory *factory,
                                   GstRTSPMedia *media,
                                   gpointer user_data);

    std::string bind_addr_;
    int port_;
    std::string mount_;
    int width_;
    int height_;
    int fps_;
    int bitrate_kbps_;
    std::string appsrc_name_;

    std::mutex mu_;
    GstRTSPMediaFactory *factory_ = nullptr;
    GstElement *appsrc_ = nullptr;
    std::atomic<bool> started_{false};
    GstClockTime timestamp_ = 0;
};
