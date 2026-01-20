#pragma once
#include <chrono>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>
#include <vector>

namespace ridersense {
using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

inline uint64_t to_nanoseconds(Timestamp ts) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             ts.time_since_epoch())
      .count();
}

inline Timestamp from_nanoseconds(uint64_t ns) {
  return Timestamp(std::chrono::nanoseconds(ns));
}

enum class FrameType { IMAGE, IMU, GPS, POSE, DETECTION, LANE };

struct Frame {
  FrameType type;
  Timestamp timestamp;
  uint64_t sequence;

  Frame(FrameType t, Timestamp ts, uint64_t seq)
      : type(t), timestamp(ts), sequence(seq) {}
  virtual ~Frame() = default;
};

struct ImageFrame : public Frame {
  cv::Mat image;
  int camera_id;

  ImageFrame(Timestamp ts, uint64_t seq, int cam_id = 0)
      : Frame(FrameType::IMAGE, ts, seq), camera_id(cam_id) {}
};

struct ImuFrame : public Frame {

  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;

  ImuFrame(Timestamp ts, uint64_t seq) : Frame(FrameType::IMU, ts, seq) {}
};

struct GpsFrame : public Frame {
  double latitude;
  double longitude;
  double altitude;
  double speed;
  double heading;
  int num_satellites;

  GpsFrame(Timestamp ts, uint64_t seq) : Frame(FrameType::GPS, ts, seq) {}
};

struct PoseFrame : public Frame {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity;

  Eigen::Matrix<double, 6, 6> covariance;

  enum class TrackingState { NOT_INITIALIZED, OK, LOST };
  TrackingState state;

  int num_tracked_points;
  PoseFrame(Timestamp ts, uint64_t seq)
      : Frame(FrameType::POSE, ts, seq), position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3d::Zero()),
        state(TrackingState::NOT_INITIALIZED), num_tracked_points(0) {
    covariance.setIdentity();
  }
};

struct DetectionFrame : public Frame {
  struct Detection {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect bbox;

    bool has_3d{false};
    Eigen::Vector3d position_3d;
    float distance;
  };

  std::vector<Detection> detections;
  int image_width;
  int image_height;

  DetectionFrame(Timestamp ts, uint64_t seq)
      : Frame(FrameType::DETECTION, ts, seq), image_width(0), image_height(0) {}
};

struct LaneFrame : public Frame {
  struct Lane {
    std::vector<cv::Point2f> points;

    float a, b, c;
    bool is_valid;
    enum class Type {
      SOLID,
      DASHED,
      UNKNOWN,

    };
    Type type;

    enum class Side { LEFT, RIGHT, CENTER };
    Side side;
  };
  std::vector<Lane> lanes;

  bool in_lane{true};
  float lateral_offset{0.0f};
  LaneFrame(Timestamp ts, uint64_t seq) : Frame(FrameType::LANE, ts, seq) {}
};

struct FusionFrame : public Frame {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d angular_velocity;

  Eigen::Matrix<double, 15, 15> covariance;

  Eigen::Vector3d accel_bias;
  Eigen::Vector3d gyro_bias;

  double lean_angle;
  double pitch_angle;
  double yaw_rate;
  double speed;
  double lateral_accel;
  double longitudinal_accel;

  double slip_angle;
  double slip_ratio;

  enum class FusionQuality { EXCELLENT, GOOD, DEGRADED, LOST };
  FusionQuality quality;

  bool has_slam{false};
  bool has_gps{false};
  bool has_imu{false};

  FusionFrame(Timestamp ts, uint64_t seq)
      : Frame(FrameType::POSE, ts, seq), position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3d::Zero()),
        angular_velocity(Eigen::Vector3d::Zero()),
        accel_bias(Eigen::Vector3d::Zero()), gyro_bias(Eigen::Vector3d::Zero()),
        lean_angle(0.0), pitch_angle(0.0), yaw_rate(0.0), speed(0.0),
        lateral_accel(0.0), longitudinal_accel(0.0), slip_angle(0.0),
        slip_ratio(0.0), quality(FusionQuality::LOST) {
    covariance.setIdentity();
  }
};

} // namespace ridersense
