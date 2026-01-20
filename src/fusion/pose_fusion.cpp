#include "fusion/pose_fusion.h"
#include <cmath>

namespace ridersense {

PoseFusion::PoseFusion(double process_noise_pos, double process_noise_vel,
                       double process_noise_orient, double slam_noise,
                       double gps_noise, double imu_noise)
    : Service("pose_fusion"), slam_noise_(slam_noise), gps_noise_(gps_noise),
      imu_noise_(imu_noise), logger_(Logger::get("fusion")) {
  ekf_.set_process_noise(process_noise_pos, process_noise_vel,
                         process_noise_orient);
}

PoseFusion::~PoseFusion() {}

bool PoseFusion::init() {
  LOG_INFO(logger_, "pose fusion initialized");
  return true;
}

bool PoseFusion::start() {
  LOG_INFO(logger_, "pose fusion started");
  return true;
}

bool PoseFusion::stop() {
  LOG_INFO(logger_, "stopping pose fusion");
  return true;
}

bool PoseFusion::shutdown() {
  LOG_INFO(logger_, "pose fusion shutdown");
  return true;
}

void PoseFusion::initialize_filter(std::shared_ptr<PoseFrame> slam_pose,
                                   std::shared_ptr<GpsFrame> gps) {
  Eigen::Vector3d init_pos;
  Eigen::Vector3d init_vel;
  Eigen::Quaterniond init_orient;

  if (slam_pose && slam_pose->state == PoseFrame::TrackingState::OK) {
    init_pos = slam_pose->position;
    init_orient = slam_pose->orientation;
    init_vel = slam_pose->velocity;
  } else if (gps) {
    init_pos = Eigen::Vector3d(0, 0, gps->altitude);
    init_orient = Eigen::Quaterniond::Identity();
    init_vel = Eigen::Vector3d(gps->speed, 0, 0); // simplified
  } else {
    init_pos.setZero();
    init_orient = Eigen::Quaterniond::Identity();
    init_vel.setZero();
  }

  ekf_.init(init_pos, init_vel, init_orient);
  initialized_ = true;

  LOG_INFO(logger_, "filter initialized at pos: ({}, {}, {})", init_pos.x(),
           init_pos.y(), init_pos.z());
}

std::shared_ptr<FusionFrame>
PoseFusion::fuse(std::shared_ptr<PoseFrame> slam_pose,
                 std::shared_ptr<ImuFrame> imu, std::shared_ptr<GpsFrame> gps) {
  auto result =
      std::make_shared<FusionFrame>(imu->timestamp, fusion_sequence_++);

  if (!initialized_ && (slam_pose || gps)) {
    initialize_filter(slam_pose, gps);
    last_update_ = imu->timestamp;
  }

  if (!initialized_) {
    result->quality = FusionFrame::FusionQuality::LOST;
    return result;
  }

  auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                   imu->timestamp - last_update_)
                   .count();
  double dt = dt_ns / 1e9;

  if (dt > 0.0 && dt < 1.0) {
    Eigen::Vector3d accel(imu->accel_x, imu->accel_y, imu->accel_z);
    Eigen::Vector3d gyro(imu->gyro_x, imu->gyro_y, imu->gyro_z);

    ekf_.predict(accel, gyro, dt);
    result->has_imu = true;
  }

  if (slam_pose && slam_pose->state == PoseFrame::TrackingState::OK) {
    Eigen::Matrix3d cov;
    cov.setIdentity();
    cov *= slam_noise_ * slam_noise_;

    ekf_.update_position(slam_pose->position, cov);
    ekf_.update_orientation(slam_pose->orientation, cov);

    result->has_slam = true;
  }

  if (gps && gps->num_satellites >= 4) {
    Eigen::Vector3d gps_pos(0, 0, gps->altitude);

    Eigen::Matrix3d pos_cov;
    pos_cov.setIdentity();
    pos_cov *= gps_noise_ * gps_noise_;

    ekf_.update_position(gps_pos, pos_cov);

    // Use GPS speed to correct velocity (heading-aligned)
    double heading_rad = gps->heading * M_PI / 180.0;
    Eigen::Vector3d gps_velocity(
        gps->speed * std::cos(heading_rad),
        gps->speed * std::sin(heading_rad),
        0.0);

    Eigen::Matrix3d vel_cov;
    vel_cov.setIdentity();
    vel_cov *= 1.0; // GPS velocity noise ~1 m/s

    ekf_.update_velocity(gps_velocity, vel_cov);

    result->has_gps = true;
  }

  result->position = ekf_.get_position();
  result->velocity = ekf_.get_velocity();
  result->orientation = ekf_.get_orientation();
  result->angular_velocity = ekf_.get_angular_velocity();
  result->accel_bias = ekf_.get_accel_bias();
  result->gyro_bias = ekf_.get_gyro_bias();
  result->covariance = ekf_.get_covariance();

  double pos_std =
      std::sqrt(result->covariance.block<3, 3>(0, 0).trace() / 3.0);
  if (pos_std < 0.1) {
    result->quality = FusionFrame::FusionQuality::EXCELLENT;
  } else if (pos_std < 0.5) {
    result->quality = FusionFrame::FusionQuality::GOOD;
  } else if (pos_std < 2.0) {
    result->quality = FusionFrame::FusionQuality::DEGRADED;
  } else {
    result->quality = FusionFrame::FusionQuality::LOST;
  }

  last_update_ = imu->timestamp;
  return result;
}

} // namespace ridersense
