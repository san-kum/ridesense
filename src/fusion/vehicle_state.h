#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"

namespace ridersense {

class VehicleState : public Service {
public:
  VehicleState(double wheelbase, double cg_height, double max_lean_angle);
  ~VehicleState() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void estimate(std::shared_ptr<FusionFrame> fusion);

private:
  double wheelbase_;
  double cg_height_;
  double max_lean_angle_;

  std::shared_ptr<spdlog::logger> logger_;
};

} // namespace ridersense
