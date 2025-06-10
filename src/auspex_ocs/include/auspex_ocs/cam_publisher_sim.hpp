#ifndef SIM_CAM_PUBLISHER_HPP
#define SIM_CAM_PUBLISHER_HPP

#include "cam_publisher_base.hpp"
#include <memory>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

/**
 * @brief Camera publisher for AirSim simulator, via RpcLibClient.
 */
class SimCamPublisher : public CamPublisherBase {
public:
    SimCamPublisher(const std::string& platform_id, const float fps);
    ~SimCamPublisher() override;

    void captureFrame() override;

private:
    std::unique_ptr<msr::airlib::RpcLibClientBase> sim_client_;
    int vhcl_id_;
};

#endif // SIM_CAM_PUBLISHER_HPP


