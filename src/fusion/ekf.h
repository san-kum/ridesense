#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ridersense {
class ExtendedKalmanFilter {
public:
  static constexpr int STATE_DIM = 16;
  ExtendedKalmanFilter();

  void init(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
            const Eigen::Quaterniond &orientation);

  void predict(const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro,
               double dt);

  void update_position(const Eigen::Vector3d &measured_position,
                       const Eigen::Matrix3d &measurement_covariance);

  void update_velocity(const Eigen::Vector3d &measured_velocity,
                       const Eigen::Matrix3d &measurement_covariance);

  void update_orientation(const Eigen::Quaterniond &measured_orientation,
                          const Eigen::Matrix3d &measurement_covariance);

  Eigen::Vector3d get_position() const { return position_; }
  Eigen::Vector3d get_velocity() const { return velocity_; }
  Eigen::Quaterniond get_orientation() const { return orientation_; }
  Eigen::Vector3d get_angular_velocity() const { return angular_velocity_; }
  Eigen::Vector3d get_accel_bias() const { return accel_bias_; }
  Eigen::Vector3d get_gyro_bias() const { return gyro_bias_; }

  Eigen::Matrix<double, 15, 15> get_covariance() const { return P_; }

  void set_process_noise(double pos, double vel, double orient);

private:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d accel_bias_;
  Eigen::Vector3d gyro_bias_;

  Eigen::Matrix<double, 15, 15> P_;

  Eigen::Matrix<double, 15, 15> Q_;

  static constexpr double GRAVITY = 9.81;
};

} // namespace ridersense
