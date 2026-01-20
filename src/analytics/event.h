#pragma once

#include <chrono>
#include <string>

namespace ridersense {

using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

enum class EventType {
  HARD_BRAKE,
  HARD_ACCEL,
  EXCESSIVE_LEAN,
  LANE_DEPARTURE,
  CLOSE_OBJECT,
  HIGH_SLIP,
};

enum class Severity { INFO, WARNING, CRITICAL };

struct Event {
  Timestamp timestamp;
  EventType type;
  Severity severity;
  std::string description;
  double value; 

  static const char *type_to_string(EventType t) {
    switch (t) {
    case EventType::HARD_BRAKE:
      return "hard_brake";
    case EventType::HARD_ACCEL:
      return "hard_accel";
    case EventType::EXCESSIVE_LEAN:
      return "excessive_lean";
    case EventType::LANE_DEPARTURE:
      return "lane_departure";
    case EventType::CLOSE_OBJECT:
      return "close_object";
    case EventType::HIGH_SLIP:
      return "high_slip";
    }
    return "unknown";
  }

  static const char *severity_to_string(Severity s) {
    switch (s) {
    case Severity::INFO:
      return "info";
    case Severity::WARNING:
      return "warning";
    case Severity::CRITICAL:
      return "critical";
    }
    return "unknown";
  }
};

} // namespace ridersense
