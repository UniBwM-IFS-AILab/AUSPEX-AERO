#ifndef CAM_PUBLISHER_BASE_HPP
#define CAM_PUBLISHER_BASE_HPP

#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "msg_context_cpp/message_loader.hpp"
#include "auspex_fci/position_listener_base.hpp"
#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

/**
 * @brief Abstract base for all camera publishers.
 */
class CamPublisherBase : public rclcpp::Node {
public:

  CamPublisherBase(const std::string& platform_id, const std::string& node_name, const float fps);
  virtual ~CamPublisherBase() = default;

  void startCapture(std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener);

  void stopCapture();

  bool isCapturing() const;

  virtual void captureFrame() = 0;

protected:

  void onTimer();

  const float fps_;
  const std::string platform_id_;
  rclcpp::Publisher<FrameData>::SharedPtr image_publisher_;
  std::shared_ptr<VehicleGlobalPositionListener_Base> gps_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex capture_mutex_;
};

#endif // CAM_PUBLISHER_BASE_HPP
