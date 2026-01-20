#pragma once

#include "analytics/event.h"
#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"
#include <deque>
#include <mutex>

namespace ridersense {

struct RidingMetrics {
  Timestamp window_start;
  Timestamp window_end;

  double max_speed{0.0};
  double avg_speed{0.0};
  double max_lean_left{0.0};
  double max_lean_right{0.0};
  double max_decel{0.0};
  double max_accel{0.0};

  uint32_t hard_brake_count{0};
  uint32_t excessive_lean_count{0};
  uint32_t lane_departure_count{0};

  uint64_t sample_count{0};
};

using MetricsCallback = std::function<void(const RidingMetrics &)>;

class MetricsCollector : public Service {
public:
  explicit MetricsCollector(double window_seconds);
  ~MetricsCollector() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void on_fusion(std::shared_ptr<FusionFrame> fusion);
  void on_event(const Event &event);

  void set_callback(MetricsCallback cb) { callback_ = cb; }

  RidingMetrics get_current_metrics() const;

private:
  void maybe_emit_window();
  void reset_window();

  double window_seconds_;
  std::shared_ptr<spdlog::logger> logger_;
  MetricsCallback callback_;

  mutable std::mutex mutex_;
  RidingMetrics current_;
  double speed_sum_{0.0};
};

} // namespace ridersense
