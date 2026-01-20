#include "fusion/vehicle_state.h"
#include <cmath>

namespace ridersense {

VehicleState::VehicleState(double wheelbase, double cg_height,
                           double max_lean_angle)
    : Service("vehicle_state"), wheelbase_(wheelbase), cg_height_(cg_height),
      max_lean_angle_(max_lean_angle), logger_(Logger::get("vehicle")) {}

VehicleState::~VehicleState() {}

bool VehicleState::init() {
  LOG_INFO(logger_, "vehicle state estimator initialized");
  LOG_INFO(logger_, "wheelbase: {} m, CG height: {} m", wheelbase_, cg_height_);
  return true;
}

bool VehicleState::start() {
  LOG_INFO(logger_, "vehicle state estimator started");
  return true;
}

bool VehicleState::stop() {
  LOG_INFO(logger_, "stopping vehicle state estimator");
  return true;
}

bool VehicleState::shutdown() {
  LOG_INFO(logger_, "vehicle state estimator shutdown");
  return true;
}

void VehicleState::estimate(std::shared_ptr<FusionFrame> fusion) {
  auto &q = fusion->orientation;

  double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  fusion->lean_angle = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1) {
    fusion->pitch_angle = std::copysign(M_PI / 2.0, sinp) * 180.0 / M_PI;
  } else {
    fusion->pitch_angle = std::asin(sinp) * 180.0 / M_PI;
  }

  fusion->yaw_rate = fusion->angular_velocity.z() * 180.0 / M_PI;

  fusion->speed = fusion->velocity.norm();

  Eigen::Vector3d accel_body = q.inverse() * fusion->velocity; // simplified
  fusion->lateral_accel = accel_body.y();
  fusion->longitudinal_accel = accel_body.x();

  if (fusion->speed > 1.0) {
    double v_lateral = fusion->velocity.y();
    double v_longitudinal = fusion->velocity.x();
    fusion->slip_angle = std::atan2(v_lateral, v_longitudinal) * 180.0 / M_PI;
  } else {
    fusion->slip_angle = 0.0;
  }

  fusion->slip_ratio = 0.0;

  fusion->lean_angle =
      std::clamp(fusion->lean_angle, -max_lean_angle_, max_lean_angle_);
}

} // namespace ridersense
