#pragma once

#include "capture/frame.h"
#include <deque>
#include <mutex>
#include <optional>

namespace ridersense {

template <typename T> class Resampler {
public:
  explicit Resampler(size_t buffer_size = 100) : buffer_size_(buffer_size) {}

  void add_sample(std::shared_ptr<T> sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(sample);

    if (buffer_.size() > buffer_size_) {
      buffer_.pop_front();
    }
  }

  std::optional<std::shared_ptr<T>> interpolate_at(Timestamp target_time) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (buffer_.size() < 2) {
      return std::nullopt;
    }

    std::shared_ptr<T> before = nullptr;
    std::shared_ptr<T> after = nullptr;

    for (size_t i = 0; i < buffer_.size() - 1; ++i) {
      if (buffer_[i]->timestamp <= target_time &&
          buffer_[i + 1]->timestamp >= target_time) {
        before = buffer_[i];
        after = buffer_[i + 1];
        break;
      }
    }

    if (!before || !after) {
      return std::nullopt;
    }

    auto dt_total = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        after->timestamp - before->timestamp)
                        .count();

    auto dt_target = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         target_time - before->timestamp)
                         .count();

    if (dt_total == 0) {
      return before;
    }

    double alpha = static_cast<double>(dt_target) / dt_total;

    return interpolate(before, after, alpha, target_time);
  }

  size_t buffer_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }

private:
  std::shared_ptr<T> interpolate(std::shared_ptr<T> before,
                                 std::shared_ptr<T> after, double alpha,
                                 Timestamp target_time);

  size_t buffer_size_;
  std::deque<std::shared_ptr<T>> buffer_;
  mutable std::mutex mutex_;
};

template <>
inline std::shared_ptr<ImuFrame>
Resampler<ImuFrame>::interpolate(std::shared_ptr<ImuFrame> before,
                                 std::shared_ptr<ImuFrame> after, double alpha,
                                 Timestamp target_time) {
  auto result = std::make_shared<ImuFrame>(target_time, 0);

  result->accel_x =
      before->accel_x + alpha * (after->accel_x - before->accel_x);
  result->accel_y =
      before->accel_y + alpha * (after->accel_y - before->accel_y);
  result->accel_z =
      before->accel_z + alpha * (after->accel_z - before->accel_z);

  result->gyro_x = before->gyro_x + alpha * (after->gyro_x - before->gyro_x);
  result->gyro_y = before->gyro_y + alpha * (after->gyro_y - before->gyro_y);
  result->gyro_z = before->gyro_z + alpha * (after->gyro_z - before->gyro_z);

  return result;
}

template <>
inline std::shared_ptr<GpsFrame>
Resampler<GpsFrame>::interpolate(std::shared_ptr<GpsFrame> before,
                                 std::shared_ptr<GpsFrame> after, double alpha,
                                 Timestamp target_time) {
  auto result = std::make_shared<GpsFrame>(target_time, 0);

  result->latitude =
      before->latitude + alpha * (after->latitude - before->latitude);
  result->longitude =
      before->longitude + alpha * (after->longitude - before->longitude);
  result->altitude =
      before->altitude + alpha * (after->altitude - before->altitude);
  result->speed = before->speed + alpha * (after->speed - before->speed);
  result->heading =
      before->heading + alpha * (after->heading - before->heading);
  result->num_satellites = before->num_satellites;

  return result;
}

} // namespace ridersense
