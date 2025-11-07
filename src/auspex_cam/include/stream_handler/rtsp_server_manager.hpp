// rtsp_server_manager.hpp
#pragma once

#include <gst/rtsp-server/rtsp-server.h>
#include <string>
#include <thread>
#include <mutex>
#include <future>
#include <iostream>

#include <atomic>

class RTSPServerManager {
public:
    static RTSPServerManager& instance(const std::string& bind_addr = "0.0.0.0", int port = 8554);
    ~RTSPServerManager();

    void add_mount(const std::string& mount, GstRTSPMediaFactory* factory);
    void remove_mount(const std::string& mount);
    
    int get_port() const { return port_; }
    std::string get_ip() const { return bind_addr_; }

    RTSPServerManager(const RTSPServerManager&) = delete;
    RTSPServerManager& operator=(const RTSPServerManager&) = delete;

private:
    RTSPServerManager(const std::string& bind_addr, int port);

    void invoke_on_loop_sync(std::function<void()> fn);

    std::string         bind_addr_;
    int                 port_{};

    std::mutex          api_mu_;
    GMainLoop*          loop_ = nullptr;
    GstRTSPServer*      server_ = nullptr;
    GstRTSPMountPoints* mounts_ = nullptr;
    std::thread         loop_thread_;
    std::thread::id     loop_thread_id_{};
    guint               attach_id_ = 0;
    std::atomic<unsigned> client_count_{0};
};
