#ifndef SIM_CAM_PUBLISHER_HPP
#define SIM_CAM_PUBLISHER_HPP

#include "cam_publisher_base.hpp"
#include "airsim_connection_manager.hpp"
#include <memory>
#include <mutex>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

/**
 * @brief Camera publisher for AirSim/Colosseum simulator
 * 
 * Uses the shared AirSimConnectionManager to prevent RPC conflicts
 * when multiple drones are running simultaneously.
 */
class SimCamPublisherUE : public CamPublisherBase {
public:
    SimCamPublisherUE(const std::string& platform_id, const int transmitHeight, const int transmitWidth, const float fps);
    ~SimCamPublisherUE() override;

    void captureFrame() override;

private:
    int vhcl_id_;
    mutable std::mutex capture_mutex_;
};

#endif // SIM_CAM_PUBLISHER_HPP


