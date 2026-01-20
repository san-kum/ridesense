#include "perception/slam_processor.h"
#include "common/logger.h"

namespace ridersense {

SlamProcessor::SlamProcessor(const std::string &vocab_path,
                             const std::string &mode)
    : Service("slam_processor"), vocab_path_(vocab_path), mode_(mode),
      logger_(Logger::get("slam")),
      state_(PoseFrame::TrackingState::NOT_INITIALIZED) {}

SlamProcessor::~SlamProcessor() {}

bool SlamProcessor::init() {
  LOG_INFO(logger_, "SLAM processor initializing");
  LOG_INFO(logger_, "Mode: {}", mode_);
  LOG_WARN(logger_, "Using stub SLAM - ORB-SLAM3 integration pending");

  // TODO: initialize ORB-SLAM3

  return true;
}

bool SlamProcessor::start() {
  LOG_INFO(logger_, "SLAM processor started");
  state_ = PoseFrame::TrackingState::OK;
  return true;
}

bool SlamProcessor::stop() {
  LOG_INFO(logger_, "Stopping SLAM processor");
  state_ = PoseFrame::TrackingState::LOST;
  return true;
}

bool SlamProcessor::shutdown() {
  LOG_INFO(logger_, "SLAM processor shutdown");
  // TODO: cleanup ORB-SLAM3
  return true;
}

std::shared_ptr<PoseFrame>
SlamProcessor::process(std::shared_ptr<ImageFrame> image,
                       std::shared_ptr<ImuFrame> imu) {
  // TODO: feed image and IMU to ORB-SLAM3
  // for now, generate dummy pose

  auto pose = std::make_shared<PoseFrame>(image->timestamp, pose_sequence_++);
  pose->state = state_;

  double t = pose_sequence_ * 0.033;
  pose->position = Eigen::Vector3d(t * 0.5, 0.0, 0.0);
  pose->orientation = Eigen::Quaterniond::Identity();
  pose->velocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  pose->num_tracked_points = 150;

  return pose;
}

} // namespace ridersense
