// rtsp_server_manager.cpp
#include "stream_handler/rtsp_server_manager.hpp"

#include <gst/gst.h>
#include <glib.h>
#include <gst/rtsp-server/rtsp-client.h>
#include <iostream>

RTSPServerManager& RTSPServerManager::instance(const std::string& bind_addr, int port) {
    static RTSPServerManager inst(bind_addr, port);
    return inst;
}

RTSPServerManager::RTSPServerManager(const std::string& bind_addr, int port)
    : bind_addr_(bind_addr), port_(port)
{
    gst_init(nullptr, nullptr);

    loop_   = g_main_loop_new(nullptr, FALSE); // default main context
    server_ = gst_rtsp_server_new();
    gst_rtsp_server_set_address(server_, bind_addr_.c_str());
    gst_rtsp_server_set_service(server_, std::to_string(port_).c_str());

    mounts_ = gst_rtsp_server_get_mount_points(server_);
    
    #ifdef DEBUG
    // Log client connects/disconnects and keep a count.
    g_signal_connect(
        server_, "client-connected",
        G_CALLBACK(+[](GstRTSPServer* /*server*/, GstRTSPClient* client, gpointer user_data) {
            auto* self = static_cast<RTSPServerManager*>(user_data);
            self->client_count_.fetch_add(1, std::memory_order_relaxed);

            // When this client closes, decrement and log.
            g_signal_connect(
                client, "closed",
                G_CALLBACK(+[](GstRTSPClient* /*client*/, gpointer user_data) {
                    auto* self = static_cast<RTSPServerManager*>(user_data);
                    self->client_count_.fetch_sub(1, std::memory_order_relaxed);
                    std::cout << "[RTSPServerManager] Client disconnected. Active clients: "
                              << self->client_count_.load(std::memory_order_relaxed) << std::endl;
                }),
                user_data);

            std::cout << "[RTSPServerManager] Client connected. Active clients: "
                      << self->client_count_.load(std::memory_order_relaxed) << std::endl;
        }),
        this);
    #endif
    
    attach_id_ = gst_rtsp_server_attach(server_, nullptr);
    if (attach_id_ == 0) {
        std::cerr << "[RTSPServerManager] Failed to attach RTSP server.\n";
    }

    // Start GLib main loop.
    loop_thread_ = std::thread([this]{
        loop_thread_id_ = std::this_thread::get_id();
        g_main_loop_run(loop_);
    });
}

void RTSPServerManager::invoke_on_loop_sync(std::function<void()> fn) {
    // If we're already on the loop thread, run directly.
    if (std::this_thread::get_id() == loop_thread_id_) {
        fn();
        return;
    }

    // Otherwise, schedule on the loop's context and wait until it runs.
    GMainContext* ctx = g_main_loop_get_context(loop_);
    auto prom = std::make_shared<std::promise<void>>();
    auto fut  = prom->get_future();

    // Move fn into heap for the C callback.
    auto* payload = new std::pair<std::function<void()>, std::shared_ptr<std::promise<void>>>(
        std::move(fn), prom
    );

    g_main_context_invoke(
        ctx,
        [](gpointer data) -> gboolean {
            auto* p = static_cast<std::pair<std::function<void()>, std::shared_ptr<std::promise<void>>>*>(data);
            (*p).first();                // run user code on the loop
            p->second->set_value();      // signal completion
            delete p;
            return G_SOURCE_REMOVE;      // return value ignored by _invoke
        },
        payload
    );

    fut.wait(); // block caller until the operation actually happened
}

void RTSPServerManager::add_mount(const std::string& mount, GstRTSPMediaFactory* factory) {
    std::string mount_ = mount;
    if (mount_.empty() || mount_[0] != '/') {
        mount_ = "/" + mount_;
    }
    std::lock_guard<std::mutex> lk(api_mu_);
    invoke_on_loop_sync([=]{
        gst_rtsp_mount_points_add_factory(mounts_, mount_.c_str(), factory);
    });
}

void RTSPServerManager::remove_mount(const std::string& mount) {
    std::string mount_ = mount;
    if (mount_.empty() || mount_[0] != '/') {
        mount_ = "/" + mount_;
    }
    std::lock_guard<std::mutex> lk(api_mu_);
    invoke_on_loop_sync([=]{
        gst_rtsp_mount_points_remove_factory(mounts_, mount_.c_str());
    });
}

RTSPServerManager::~RTSPServerManager() {
    // Block new API calls and serialize with any in-flight ones.
    std::lock_guard<std::mutex> lk(api_mu_);

    if (loop_) g_main_loop_quit(loop_);
    if (loop_thread_.joinable()) loop_thread_.join();

    if (attach_id_ != 0) {
        g_source_remove(attach_id_);
        attach_id_ = 0;
    }

    if (mounts_) { g_object_unref(mounts_); mounts_ = nullptr; }
    if (server_) { g_object_unref(server_); server_ = nullptr; }
    if (loop_)   { g_main_loop_unref(loop_); loop_ = nullptr; }
}