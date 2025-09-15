#include "auspex_ocs/cam_publisher_empty.hpp"

EmptyCamPublisher::EmptyCamPublisher(const std::string& platform_id, const float fps)
    : CamPublisherBase(platform_id, "empty_cam_publisher", fps)
{
}

EmptyCamPublisher::~EmptyCamPublisher() {
    stopCapture();
}

void EmptyCamPublisher::captureFrame() {
    // No operation
}