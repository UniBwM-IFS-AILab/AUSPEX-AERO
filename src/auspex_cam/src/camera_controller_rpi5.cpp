#include "auspex_cam/camera_controller_rpi5.hpp"

CameraControllerRPI5::CameraControllerRPI5(const std::string& platform_id,
                                           const std::string& ip_addr, int port,
                                           int stream_width_color, int stream_height_color, int stream_bitrate_color,
                                           int /*stream_width_ir*/, int /*stream_height_ir*/, int /*stream_bitrate_ir*/,
                                           const float fps)
: CameraControllerBase(platform_id, "camera_controller_rpi5",
                       ip_addr, port,
                       stream_width_color, stream_height_color, stream_bitrate_color,
                       /*ir w*/0, /*ir h*/0, /*ir br*/0,
                       fps)
, out_w_(stream_width_color)
, out_h_(stream_height_color)
, fps_(fps)
{
    initCamera();
    startStreaming();
}

CameraControllerRPI5::~CameraControllerRPI5() {
    stopStreaming();
    shutdownCamera();
}

bool CameraControllerRPI5::check_if_connected() {
    return connected_.load();
}

CameraControllerBase::Capabilities CameraControllerRPI5::available_capabilities() {
    return Capabilities{ true, false, false, false };
}

std::optional<cv::Mat> CameraControllerRPI5::capture_color_image() {
    std::lock_guard<std::mutex> lk(latest_mtx_);
    if (latest_rgb_.empty()) return std::nullopt;
    return std::optional<cv::Mat>(latest_rgb_.clone()); // immediate, non-blocking
}

void CameraControllerRPI5::initCamera() {
    cam_mgr_ = std::make_unique<libcamera::CameraManager>();
    if (cam_mgr_->start() < 0)
        throw std::runtime_error("CameraManager start failed");

    if (cam_mgr_->cameras().empty())
        throw std::runtime_error("No cameras found");

    camera_ = cam_mgr_->cameras()[0];
    if (!camera_ || camera_->acquire() < 0)
        throw std::runtime_error("Camera acquire failed");

    // Use the ISP: Viewfinder stream in RGB888 for simple, high-quality output.
    config_ = camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
    if (!config_ || config_->size() != 1)
        throw std::runtime_error("Failed to generate configuration");

    auto &cfg = config_->at(0);
    cfg.pixelFormat = libcamera::formats::RGB888;
    // Ask for your requested output size; ISP will choose the nearest mode and scale.
    if (out_w_ > 0 && out_h_ > 0) {
        cfg.size.width  = out_w_;
        cfg.size.height = out_h_;
    }
    // A small buffer queue helps with jitter.
    cfg.bufferCount = std::max(3u, cfg.bufferCount);

    if (config_->validate() == libcamera::CameraConfiguration::Invalid)
        throw std::runtime_error("Invalid camera configuration");

    if (camera_->configure(config_.get()) < 0)
        throw std::runtime_error("Camera configure failed");

    stream_ = cfg.stream();

    // Allocate buffers
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0)
        throw std::runtime_error("Buffer allocation failed");

    auto &bufs = allocator_->buffers(stream_);
    buffers_.reserve(bufs.size());
    mapped_.reserve(bufs.size());

    for (auto &fbp : bufs) {
        libcamera::FrameBuffer *fb = fbp.get();
        buffers_.push_back(fb);

        if (fb->planes().size() != 1)
            throw std::runtime_error("Unexpected multi-plane RGB output");

        const auto &pl = fb->planes()[0];
        void *addr = ::mmap(nullptr, pl.length, PROT_READ | PROT_WRITE, MAP_SHARED, pl.fd.get(), pl.offset);
        if (addr == MAP_FAILED)
            throw std::runtime_error("mmap failed");

        Mapped mb;
        mb.addr         = addr;
        mb.length       = pl.length;
        mb.width        = cfg.size.width;
        mb.height       = cfg.size.height;
        // Step/stride in bytes per line. If the driver pads, this captures it.
        mb.bytesPerLine = static_cast<int>(pl.length / cfg.size.height);
        mapped_.push_back(mb);
    }

    // Hook request completion
    camera_->requestCompleted.connect(this, &CameraControllerRPI5::onRequestCompleted_);

    connected_.store(true);
}

void CameraControllerRPI5::shutdownCamera() {
    connected_.store(false);

    // Unmap
    for (auto &mb : mapped_) {
        if (mb.addr && mb.length) ::munmap(mb.addr, mb.length);
        mb.addr = nullptr; mb.length = 0;
    }
    mapped_.clear();
    buffers_.clear();
    requests_.clear();
    allocator_.reset();

    if (camera_) {
        camera_->release();
        camera_.reset();
    }
    if (cam_mgr_) {
        cam_mgr_->stop();
        cam_mgr_.reset();
    }

    {
        std::lock_guard<std::mutex> lk(latest_mtx_);
        latest_rgb_.release();
    }
}

void CameraControllerRPI5::startStreaming() {
    if (!camera_) return;

    // One request per buffer
    requests_.reserve(buffers_.size());
    for (auto *fb : buffers_) {
        auto req = camera_->createRequest();
        if (!req) throw std::runtime_error("createRequest failed");
        if (req->addBuffer(stream_, fb) < 0)
            throw std::runtime_error("addBuffer failed");
        requests_.push_back(std::move(req));
    }

    if (camera_->start() < 0)
        throw std::runtime_error("camera start failed");

    for (auto &req : requests_) {
        if (camera_->queueRequest(req.get()) < 0)
            throw std::runtime_error("queueRequest failed");
    }
}

void CameraControllerRPI5::stopStreaming() {
    if (!camera_) return;
    camera_->stop();
}

void CameraControllerRPI5::onRequestCompleted_(libcamera::Request *req) {
    // Re-queue immediately to keep the pipeline full.
    libcamera::FrameBuffer *fb = nullptr;
    if (req->status() == libcamera::Request::RequestComplete && !req->buffers().empty()) {
        fb = req->buffers().begin()->second;
    }

    // Convert to cv::Mat and stash the latest frame.
    if (fb) {
        int idx = -1;
        for (size_t i = 0; i < buffers_.size(); ++i) {
            if (buffers_[i] == fb) { idx = static_cast<int>(i); break; }
        }
        if (idx >= 0) {
            const Mapped &mb = mapped_[static_cast<size_t>(idx)];
            // Wrap the RGB buffer directly with step = bytesPerLine and clone.
            cv::Mat rgb_view(mb.height, mb.width, CV_8UC3, mb.addr, static_cast<size_t>(mb.bytesPerLine));
            {
                std::lock_guard<std::mutex> lk(latest_mtx_);
                if (out_w_ > 0 && out_h_ > 0 &&
                    (rgb_view.cols != out_w_ || rgb_view.rows != out_h_)) {
                    cv::resize(rgb_view, latest_rgb_, cv::Size(out_w_, out_h_), 0, 0, cv::INTER_LINEAR);
                } else {
                    latest_rgb_ = rgb_view.clone(); // copy and keep in RGB order
                }
            }
        }
    }

    req->reuse();
    if (fb) req->addBuffer(stream_, fb);
    camera_->queueRequest(req);
}