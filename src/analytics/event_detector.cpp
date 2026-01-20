#include "analytics/event_detector.h"
#include <cmath>
#include <sstream>

namespace ridersense {

EventDetector::EventDetector(const EventDetectorConfig &config)
    : Service("event_detector"), config_(config),
      logger_(Logger::get("events")) {}

EventDetector::~EventDetector() {}

bool EventDetector::init() {
  LOG_INFO(logger_, "event detector initialized");
  LOG_INFO(logger_, "thresholds - brake: {:.1f}, accel: {:.1f}, lean: {:.1f}",
           config_.hard_brake_threshold, config_.hard_accel_threshold,
           config_.max_lean_threshold);
  return true;
}

bool EventDetector::start() {
  LOG_INFO(logger_, "event detector started");
  return true;
}

bool EventDetector::stop() {
  LOG_INFO(logger_, "stopping event detector");
  return true;
}

bool EventDetector::shutdown() {
  LOG_INFO(logger_, "event detector shutdown");
  return true;
}

bool EventDetector::can_emit(EventType type) {
  auto now = std::chrono::high_resolution_clock::now();
  auto it = last_emit_.find(type);
  if (it == last_emit_.end()) {
    return true;
  }

  auto elapsed = std::chrono::duration<double>(now - it->second).count();
  return elapsed >= config_.cooldown_seconds;
}

void EventDetector::emit(EventType type, Severity severity,
                         const std::string &desc, double value) {
  if (!can_emit(type)) {
    return;
  }

  auto now = std::chrono::high_resolution_clock::now();
  last_emit_[type] = now;

  Event event{now, type, severity, desc, value};

  LOG_WARN(logger_, "[{}] {} - {} (value: {:.2f})",
           Event::severity_to_string(severity), Event::type_to_string(type),
           desc, value);

  if (callback_) {
    callback_(event);
  }
}

void EventDetector::process(std::shared_ptr<FusionFrame> fusion,
                            std::shared_ptr<DetectionFrame> detections,
                            std::shared_ptr<LaneFrame> lanes) {
  if (!fusion) {
    return;
  }

  if (fusion->longitudinal_accel < config_.hard_brake_threshold) {
    Severity sev = fusion->longitudinal_accel < config_.hard_brake_threshold * 1.5
                       ? Severity::CRITICAL
                       : Severity::WARNING;
    std::ostringstream oss;
    oss << "deceleration " << std::abs(fusion->longitudinal_accel) << " m/s^2";
    emit(EventType::HARD_BRAKE, sev, oss.str(), fusion->longitudinal_accel);
  }

  if (fusion->longitudinal_accel > config_.hard_accel_threshold) {
    emit(EventType::HARD_ACCEL, Severity::INFO, "rapid acceleration",
         fusion->longitudinal_accel);
  }

  double abs_lean = std::abs(fusion->lean_angle);
  if (abs_lean > config_.max_lean_threshold) {
    Severity sev =
        abs_lean > config_.max_lean_threshold * 1.2 ? Severity::CRITICAL : Severity::WARNING;
    std::ostringstream oss;
    oss << "lean angle " << abs_lean << " degrees";
    emit(EventType::EXCESSIVE_LEAN, sev, oss.str(), fusion->lean_angle);
  }

  double abs_slip = std::abs(fusion->slip_angle);
  if (abs_slip > config_.high_slip_threshold) {
    emit(EventType::HIGH_SLIP, Severity::WARNING, "high slip angle", fusion->slip_angle);
  }

  if (lanes && !lanes->in_lane) {
    emit(EventType::LANE_DEPARTURE, Severity::WARNING, "lane departure detected",
         lanes->lateral_offset);
  }

  if (detections) {
    for (const auto &det : detections->detections) {
      if (det.has_3d && det.distance < config_.close_object_distance) {
        std::ostringstream oss;
        oss << det.class_name << " at " << det.distance << "m";
        emit(EventType::CLOSE_OBJECT, Severity::WARNING, oss.str(), det.distance);
        break; 
      }
    }
  }
}

} // namespace ridersense
