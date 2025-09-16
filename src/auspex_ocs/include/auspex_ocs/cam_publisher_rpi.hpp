#ifndef RPI_CAM_PUBLISHER_HPP
#define RPI_CAM_PUBLISHER_HPP

#include "cam_publisher_base.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @brief Camera publisher for Raspberry Pi using OpenCV capture.
 */
class RPICamPublisher : public CamPublisherBase {
public:

    RPICamPublisher(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps);
    ~RPICamPublisher() override;

    void captureFrame() override;

private:

    void initCamera();

    cv::VideoCapture cap_;
};

#endif // RPI_CAM_PUBLISHER_HPP