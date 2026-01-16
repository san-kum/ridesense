#pragma once
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>

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

// TODO: add PoseFrame, DetectionFrame, LaneFrame later

} // namespace ridersense
