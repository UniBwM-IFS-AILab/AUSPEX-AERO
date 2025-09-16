#include "auspex_ocs/cam_publisher_rpi5.hpp"

RPI5CamPublisher::RPI5CamPublisher(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps)
    : CamPublisherBase(platform_id, "rpi5_cam_publisher", transmitHeight, transmitWidth, fps),
      buffer_(nullptr)
{
    // initialize libcamera
    initCamera();
}

RPI5CamPublisher::~RPI5CamPublisher() {
    // ensure we stop the timer
    stopCapture();
    if (camera_) {
        camera_->stop();
        camera_->release();
    }
    if (cameraManager_) {
        cameraManager_->stop();
    }
}

void RPI5CamPublisher::initCamera() {
    cameraManager_ = std::make_unique<libcamera::CameraManager>();
    wb_ = cv::xphoto::createSimpleWB();

    if (cameraManager_->start() != 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to start Camera Manager.");
        throw std::runtime_error("Camera Manager start failed");
    }

    if (cameraManager_->cameras().empty()){
        RCLCPP_ERROR(this->get_logger(), "No cameras found.");
        throw std::runtime_error("No cameras found.");
    }

    camera_ = cameraManager_->cameras()[0];
    if (!camera_ || camera_->acquire()){
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera.");
        throw std::runtime_error("Camera acquisition failed.");
    }

    config_ = camera_->generateConfiguration({libcamera::StreamRole::Raw});
    config_->at(0).size = {2028,1080};
    config_->at(0).pixelFormat = libcamera::formats::SBGGR12;
    config_->at(0).stride = 2048;

    if (config_->validate() == libcamera::CameraConfiguration::Invalid){
        RCLCPP_ERROR(this->get_logger(), "Invalid camera configuration.");
        throw std::runtime_error("Invalid camera configuration.");
    }

    if (camera_->configure(config_.get())){
        RCLCPP_ERROR(this->get_logger(), "Failed to configure camera.");
        throw std::runtime_error("Camera configuration failed.");
    }

    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(config_->at(0).stream()) < 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffers.");
        throw std::runtime_error("Buffer allocation failed.");
    }

    auto &buffers = allocator_->buffers(config_->at(0).stream());
    if (buffers.empty()){
        RCLCPP_ERROR(this->get_logger(), "No buffers allocated.");
        throw std::runtime_error("No buffers allocated.");
    }

    buffer_ = buffers[0].get();

    request_ = camera_->createRequest();
    if (!request_){
        RCLCPP_ERROR(this->get_logger(), "Failed to create request.");
        return;
    }
    if (!buffer_) {
        RCLCPP_ERROR(this->get_logger(), "Buffer is null.");
        return;
    }

    if (request_->addBuffer(config_->at(0).stream(), buffer_) < 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to attach buffer to request.");
        return;
    }


    if (camera_->start()){
        RCLCPP_ERROR(this->get_logger(), "Failed to start the camera.");
        throw std::runtime_error("Failed to start the camera.");
    }

    RCLCPP_INFO(this->get_logger(), "Camera initialized successfully.");
}

void RPI5CamPublisher::captureFrame() {
    // Queue & wait
    if (camera_->queueRequest(request_.get()) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to queue libcamera request");
        return;
    }

    while(request_->status() != libcamera::Request::RequestComplete){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(request_->status() ==libcamera::Request::RequestCancelled){
            RCLCPP_WARN(this->get_logger(), "Request was cancelled.");
            return;
        }
    }

    libcamera::FrameBuffer *buffer = request_->buffers().begin()->second;
    if (!buffer) {
        RCLCPP_ERROR(this->get_logger(), "Request contains no buffers.");
        return;
    }
    const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];

    void *data = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), plane.offset);
    if (data == MAP_FAILED){
        RCLCPP_ERROR(this->get_logger(), "Failed to mmap buffer.");
        return;
    }

    uint8_t *rawData = static_cast<uint8_t *>(data);

    cv::Mat bayerImage(config_->at(0).size.height, config_->at(0).size.width, CV_16UC1);
    for (uint y = 0; y < config_->at(0).size.height; ++y){
        memcpy(bayerImage.ptr(y), rawData + y * config_->at(0).stride, config_->at(0).size.width * 2);
    }

    bayerImage.convertTo(bayerImage, CV_8UC1, 1.0 / 256.0);
    cv::cvtColor(bayerImage, bayerImage, cv::COLOR_BayerBG2RGB);
    cv::resize(bayerImage, bayerImage, cv::Size(transmitWidth_, transmitHeight_), 0, 0, cv::INTER_LINEAR);

    wb_->balanceWhite(bayerImage, bayerImage);

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);
    std::vector<uint8_t> jpg;
    if (!cv::imencode(".jpg", bayerImage, jpg, compression_params)) {
        RCLCPP_ERROR(get_logger(), "JPG encoding failed");
        return;
    }

    auto image_msg = FrameData();
    auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    compressed_image_msg->header = std_msgs::msg::Header();
    compressed_image_msg->format = "jpeg";
    compressed_image_msg->data = jpg;
    image_msg.image_compressed = *compressed_image_msg;
    image_msg.fps = int(fps_);
    image_msg.res_width = transmitWidth_;
    image_msg.res_height = transmitHeight_;

    if (gps_listener_) {
        auto g = gps_listener_->get_recent_gps_msg();
        image_msg.gps_position.latitude  = g->latitude_deg;
        image_msg.gps_position.longitude = g->longitude_deg;
        image_msg.gps_position.altitude  = g->absolute_altitude_m;
    }

    this->image_publisher_->publish(image_msg);

    //Unmap buffer etc
    munmap(data, plane.length);
    request_->reuse();
    if (request_->addBuffer(config_->at(0).stream(), buffer_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reattach buffer to the request.");
        return;
    }
}
