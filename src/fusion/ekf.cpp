#include "fusion/ekf.h"

namespace ridersense {

ExtendedKalmanFilter::ExtendedKalmanFilter() {
  position_.setZero();
  velocity_.setZero();
  orientation_ = Eigen::Quaterniond::Identity();
  angular_velocity_.setZero();
  accel_bias_.setZero();
  gyro_bias_.setZero();

  P_.setIdentity();
  P_ *= 1.0;

  Q_.setIdentity();
  Q_ *= 0.01;
}

void ExtendedKalmanFilter::init(const Eigen::Vector3d &position,
                                const Eigen::Vector3d &velocity,
                                const Eigen::Quaterniond &orientation) {
  position_ = position;
  velocity_ = velocity;
  orientation_ = orientation;
  orientation_.normalize();
}

void ExtendedKalmanFilter::set_process_noise(double pos, double vel,
                                             double orient) {
  Q_.setIdentity();
  Q_.block<3, 3>(0, 0) *= pos * pos;       // position
  Q_.block<3, 3>(3, 3) *= vel * vel;       // velocity
  Q_.block<3, 3>(6, 6) *= orient * orient; // orientation error
  Q_.block<3, 3>(9, 9) *= 0.1;             // angular velocity
  Q_.block<3, 3>(12, 12) *= 0.001;         // accel bias

}

void ExtendedKalmanFilter::predict(const Eigen::Vector3d &accel,
                                   const Eigen::Vector3d &gyro, double dt) {
  Eigen::Vector3d accel_unbiased = accel - accel_bias_;
  Eigen::Vector3d gyro_unbiased = gyro - gyro_bias_;

  Eigen::Vector3d accel_world = orientation_ * accel_unbiased;
  accel_world.z() -= GRAVITY;

  position_ += velocity_ * dt + 0.5 * accel_world * dt * dt;
  velocity_ += accel_world * dt;

  double gyro_norm = gyro_unbiased.norm();
  if (gyro_norm > 1e-6) {
    Eigen::Vector3d axis = gyro_unbiased / gyro_norm;
    double angle = gyro_norm * dt;
    Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
    orientation_ = orientation_ * delta_q;
    orientation_.normalize();
  }

  angular_velocity_ = gyro_unbiased;

  P_ = P_ + Q_ * dt;
}

void ExtendedKalmanFilter::update_position(
    const Eigen::Vector3d &measured_position,
    const Eigen::Matrix3d &measurement_covariance) {
  Eigen::Vector3d y = measured_position - position_;

  Eigen::Matrix<double, 3, 15> H;
  H.setZero();
  H.block<3, 3>(0, 0).setIdentity();

  Eigen::Matrix3d S = H * P_ * H.transpose() + measurement_covariance;

  Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();

  Eigen::Matrix<double, 15, 1> dx = K * y;
  position_ += dx.segment<3>(0);
  velocity_ += dx.segment<3>(3);

  Eigen::Vector3d orient_error = dx.segment<3>(6);
  double error_norm = orient_error.norm();
  if (error_norm > 1e-6) {
    Eigen::Quaterniond error_q(
        Eigen::AngleAxisd(error_norm, orient_error / error_norm));
    orientation_ = orientation_ * error_q;
    orientation_.normalize();
  }

  angular_velocity_ += dx.segment<3>(9);
  accel_bias_ += dx.segment<3>(12);

  Eigen::Matrix<double, 15, 15> I;
  I.setIdentity();
  P_ = (I - K * H) * P_;
}

void ExtendedKalmanFilter::update_velocity(
    const Eigen::Vector3d &measured_velocity,
    const Eigen::Matrix3d &measurement_covariance) {
  Eigen::Vector3d y = measured_velocity - velocity_;

  Eigen::Matrix<double, 3, 15> H;
  H.setZero();
  H.block<3, 3>(0, 3).setIdentity();

  Eigen::Matrix3d S = H * P_ * H.transpose() + measurement_covariance;
  Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();

  Eigen::Matrix<double, 15, 1> dx = K * y;
  position_ += dx.segment<3>(0);
  velocity_ += dx.segment<3>(3);

  Eigen::Matrix<double, 15, 15> I;
  I.setIdentity();
  P_ = (I - K * H) * P_;
}

void ExtendedKalmanFilter::update_orientation(
    const Eigen::Quaterniond &measured_orientation,
    const Eigen::Matrix3d &measurement_covariance) {
  Eigen::Quaterniond q_error = orientation_.inverse() * measured_orientation;

  Eigen::Vector3d error_axis(q_error.x(), q_error.y(), q_error.z());
  double error_angle = 2.0 * std::atan2(error_axis.norm(), q_error.w());

  if (error_axis.norm() > 1e-6) {
    error_axis.normalize();
    error_axis *= error_angle;
  }

  Eigen::Matrix<double, 3, 15> H;
  H.setZero();
  H.block<3, 3>(0, 6).setIdentity();

  Eigen::Matrix3d S = H * P_ * H.transpose() + measurement_covariance;
  Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();

  Eigen::Matrix<double, 15, 1> dx = K * error_axis;

  Eigen::Vector3d orient_correction = dx.segment<3>(6);
  double corr_norm = orient_correction.norm();
  if (corr_norm > 1e-6) {
    Eigen::Quaterniond corr_q(
        Eigen::AngleAxisd(corr_norm, orient_correction / corr_norm));
    orientation_ = orientation_ * corr_q;
    orientation_.normalize();
  }

  Eigen::Matrix<double, 15, 15> I;
  I.setIdentity();
  P_ = (I - K * H) * P_;
}

} // namespace ridersense
