#include "analytics/metrics_collector.h"
#include <algorithm>

namespace ridersense {

MetricsCollector::MetricsCollector(double window_seconds)
    : Service("metrics_collector"), window_seconds_(window_seconds),
      logger_(Logger::get("metrics")) {}

MetricsCollector::~MetricsCollector() {}

bool MetricsCollector::init() {
  LOG_INFO(logger_, "metrics collector initialized (window: {}s)",
           window_seconds_);
  reset_window();
  return true;
}

bool MetricsCollector::start() {
  LOG_INFO(logger_, "metrics collector started");
  return true;
}

bool MetricsCollector::stop() {
  LOG_INFO(logger_, "stopping metrics collector");
  if (current_.sample_count > 0 && callback_) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_.window_end = std::chrono::high_resolution_clock::now();
    current_.avg_speed = speed_sum_ / current_.sample_count;
    callback_(current_);
  }
  return true;
}

bool MetricsCollector::shutdown() {
  LOG_INFO(logger_, "metrics collector shutdown");
  return true;
}

void MetricsCollector::reset_window() {
  current_ = RidingMetrics{};
  current_.window_start = std::chrono::high_resolution_clock::now();
  speed_sum_ = 0.0;
}

void MetricsCollector::maybe_emit_window() {
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed =
      std::chrono::duration<double>(now - current_.window_start).count();

  if (elapsed >= window_seconds_ && current_.sample_count > 0) {
    current_.window_end = now;
    current_.avg_speed = speed_sum_ / current_.sample_count;

    LOG_INFO(logger_,
             "window metrics - samples: {}, max_speed: {:.1f}, avg_speed: "
             "{:.1f}, max_lean: {:.1f}/{:.1f}",
             current_.sample_count, current_.max_speed, current_.avg_speed,
             current_.max_lean_left, current_.max_lean_right);

    if (callback_) {
      callback_(current_);
    }

    reset_window();
  }
}

void MetricsCollector::on_fusion(std::shared_ptr<FusionFrame> fusion) {
  if (!fusion)
    return;

  std::lock_guard<std::mutex> lock(mutex_);

  current_.sample_count++;
  speed_sum_ += fusion->speed;
  current_.max_speed = std::max(current_.max_speed, fusion->speed);

  if (fusion->lean_angle < 0) {
    current_.max_lean_left =
        std::max(current_.max_lean_left, std::abs(fusion->lean_angle));
  } else {
    current_.max_lean_right =
        std::max(current_.max_lean_right, fusion->lean_angle);
  }

  current_.max_decel =
      std::min(current_.max_decel, fusion->longitudinal_accel);
  current_.max_accel =
      std::max(current_.max_accel, fusion->longitudinal_accel);

  maybe_emit_window();
}

void MetricsCollector::on_event(const Event &event) {
  std::lock_guard<std::mutex> lock(mutex_);

  switch (event.type) {
  case EventType::HARD_BRAKE:
    current_.hard_brake_count++;
    break;
  case EventType::EXCESSIVE_LEAN:
    current_.excessive_lean_count++;
    break;
  case EventType::LANE_DEPARTURE:
    current_.lane_departure_count++;
    break;
  default:
    break;
  }
}

RidingMetrics MetricsCollector::get_current_metrics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  RidingMetrics copy = current_;
  if (copy.sample_count > 0) {
    copy.avg_speed = speed_sum_ / copy.sample_count;
  }
  return copy;
}

} // namespace ridersense
