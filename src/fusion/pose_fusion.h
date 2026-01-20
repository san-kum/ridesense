#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include "fusion/ekf.h"
#include "services/service.h"

namespace ridersense {

// fuses SLAM, IMU, GPS into unified pose estimate
class PoseFusion : public Service {
public:
  PoseFusion(double process_noise_pos, double process_noise_vel,
             double process_noise_orient, double slam_noise, double gps_noise,
             double imu_noise);
  ~PoseFusion() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  std::shared_ptr<FusionFrame> fuse(std::shared_ptr<PoseFrame> slam_pose,
                                    std::shared_ptr<ImuFrame> imu,
                                    std::shared_ptr<GpsFrame> gps);

private:
  void initialize_filter(std::shared_ptr<PoseFrame> slam_pose,
                         std::shared_ptr<GpsFrame> gps);

  ExtendedKalmanFilter ekf_;

  double slam_noise_;
  double gps_noise_;
  double imu_noise_;

  std::shared_ptr<spdlog::logger> logger_;

  bool initialized_{false};
  Timestamp last_update_;
  uint64_t fusion_sequence_{0};
};

} // namespace ridersense
