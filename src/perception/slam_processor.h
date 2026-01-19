#pragma once

#include "capture/frame.h"
#include "services/service.h"
#include <memory>
#include <spdlog/logger.h>
#include <string>
namespace ridersense {

// TODO: integrate actual ORB-SLAM later
class SlamProcessor : public Service {
public:
  SlamProcessor(const std::string &vocab_path, const std::string &mode);
  ~SlamProcessor() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  std::shared_ptr<PoseFrame> process(std::shared_ptr<ImageFrame> image,
                                     std::shared_ptr<ImuFrame> imu);
  PoseFrame::TrackingState get_state() const { return state_; }

private:
  std::string vocab_path_;
  std::string mode_;
  std::shared_ptr<spdlog::logger> logger_;

  PoseFrame::TrackingState state_;
  uint64_t pose_sequence_{0};

  // TODO: add ORB-SLAM sys pointer
};

} // namespace ridersense
