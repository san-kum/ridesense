#pragma once

#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"
#include "sync/resampler.h"
#include "sync/timestamp_sync.h"
#include <deque>
#include <functional>

namespace ridersense {

struct SyncedFrames {
  Timestamp timestamp;
  std::shared_ptr<ImageFrame> image;
  std::shared_ptr<ImuFrame> imu;
  std::shared_ptr<GpsFrame> gps;
};

using SyncedCallback = std::function<void(const SyncedFrames &)>;

class TimeAligner : public Service {
public:
  TimeAligner(std::chrono::milliseconds max_time_diff, size_t buffer_size);
  ~TimeAligner() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void on_image(std::shared_ptr<ImageFrame> frame);
  void on_imu(std::shared_ptr<ImuFrame> frame);
  void on_gps(std::shared_ptr<GpsFrame> frame);

  void set_callback(SyncedCallback cb) { callback_ = cb; }

  struct Stats {
    uint64_t images_received{0};
    uint64_t imu_received{0};
    uint64_t gps_received{0};
    uint64_t bundles_emitted{0};
    uint64_t images_dropped{0};
  };
  Stats get_stats() const;

private:
  void try_emit();

  std::chrono::milliseconds max_time_diff_;
  size_t buffer_size_;

  std::shared_ptr<spdlog::logger> logger_;

  TimestampSync ts_sync_;
  Resampler<ImuFrame> imu_resampler_;
  Resampler<GpsFrame> gps_resampler_;

  std::deque<std::shared_ptr<ImageFrame>> image_buffer_;
  mutable std::mutex buffer_mutex_;

  SyncedCallback callback_;
  Stats stats_;
};

} // namespace ridersense
