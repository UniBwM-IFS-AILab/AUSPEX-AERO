#ifndef SIM_CAM_PUBLISHER_HPP
#define SIM_CAM_PUBLISHER_HPP

#include "cam_publisher_base.hpp"
#include <memory>

/**
 * @brief Camera publisher if no camera specified
 */
class EmptyCamPublisher : public CamPublisherBase {
public:
    EmptyCamPublisher(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps = 30.0f);
    ~EmptyCamPublisher() override;

    void captureFrame() override;
};

#endif // SIM_CAM_PUBLISHER_HPP


