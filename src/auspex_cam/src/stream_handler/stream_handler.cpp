// stream_handler.cpp
#include "stream_handler/stream_handler.hpp"

#include <iostream>
#include <sstream>
#include <cstring>

StreamHandler::StreamHandler(const std::string &bind_addr, int port,
                             const std::string &mount, int width, int height,
                             int fps, int bitrate_kbps)
    : bind_addr_(bind_addr), port_(port), mount_(mount),
      width_(width), height_(height), fps_(fps), bitrate_kbps_(bitrate_kbps) {
    appsrc_name_ = "appsrc_" + mount_.substr(1);
}

StreamHandler::~StreamHandler() { stop(); }

void StreamHandler::start() {
    if (started_) return;

    RTSPServerManager::instance(bind_addr_, port_);
    bind_addr_ = RTSPServerManager::instance().get_ip();
    port_ = RTSPServerManager::instance().get_port();

    std::ostringstream launch;
    launch << "( appsrc name=" << appsrc_name_
           << " is-live=true format=time do-timestamp=true"
           << " caps=video/x-raw,format=BGR,width=" << width_ << ",height=" << height_
           << ",framerate=" << fps_ << "/1"
           << " ! videoconvert ! video/x-raw,format=I420"
           << " ! x264enc tune=zerolatency bitrate=" << bitrate_kbps_
           << " speed-preset=ultrafast key-int-max=" << fps_ * 2
           << " ! rtph264pay name=pay0 pt=96 config-interval=1 )";

    factory_ = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory_, launch.str().c_str());
    gst_rtsp_media_factory_set_latency(factory_, 0);
    gst_rtsp_media_factory_set_shared(factory_, TRUE);

    g_signal_connect(factory_, "media-configure", (GCallback)on_media_configure, this);
    RTSPServerManager::instance().add_mount(mount_, factory_);
    started_ = true;
}

void StreamHandler::stop() {
    if (!started_) return;
    RTSPServerManager::instance().remove_mount(mount_);
    std::lock_guard<std::mutex> lk(mu_);
    if (appsrc_) { g_object_unref(appsrc_); appsrc_ = nullptr; }
    if (factory_) { g_object_unref(factory_); factory_ = nullptr; }
    started_ = false;
}

bool StreamHandler::submit_frame(const cv::Mat &frame) {
    if (!started_) return false;

    cv::Mat bgr;
    if (frame.cols != width_ || frame.rows != height_) {
        cv::resize(frame, bgr, cv::Size(width_, height_));
    } else {
        bgr = frame;
    }

    std::lock_guard<std::mutex> lk(mu_);
    if (!appsrc_) return false;

    const size_t size = static_cast<size_t>(bgr.total() * bgr.elemSize());
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    if (!buffer) return false;

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        std::memcpy(map.data, bgr.data, size);
        gst_buffer_unmap(buffer, &map);
    } else {
        gst_buffer_unref(buffer);
        return false;
    }

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (ret != GST_FLOW_OK) {
        return false;
    }
    return true;
}

void StreamHandler::on_media_configure(GstRTSPMediaFactory *, GstRTSPMedia *media, gpointer user_data) {
    auto *self = static_cast<StreamHandler *>(user_data);
    GstElement *element = gst_rtsp_media_get_element(media);
    GstElement *appsrc = gst_bin_get_by_name(GST_BIN(element), self->appsrc_name_.c_str());
    if (!appsrc) {
        std::cerr << "[StreamHandler] Failed to get appsrc: " << self->appsrc_name_ << "\n";
    } else {
        std::lock_guard<std::mutex> lk(self->mu_);
        if (self->appsrc_) g_object_unref(self->appsrc_);
        self->appsrc_ = appsrc;
        g_object_set(G_OBJECT(self->appsrc_),
                    "is-live", TRUE,
                    "do-timestamp", TRUE,
                    nullptr);
    }
    gst_object_unref(element);
}
