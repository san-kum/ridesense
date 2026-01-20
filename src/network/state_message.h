#pragma once

#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>

namespace ridersense {

enum class MessageType { STATE_UPDATE, EVENT, METRICS, HEARTBEAT };

struct StateMessage {
  MessageType type;
  uint64_t timestamp_ms;
  std::string payload;

  static const char *type_to_string(MessageType t) {
    switch (t) {
    case MessageType::STATE_UPDATE:
      return "state_update";
    case MessageType::EVENT:
      return "event";
    case MessageType::METRICS:
      return "metrics";
    case MessageType::HEARTBEAT:
      return "heartbeat";
    }
    return "unknown";
  }

  std::string to_json() const {
    std::ostringstream oss;
    oss << "{\"type\":\"" << type_to_string(type) << "\",\"ts\":" << timestamp_ms;
    if (!payload.empty()) {
      oss << "," << payload;
    }
    oss << "}";
    return oss.str();
  }

  static uint64_t now_ms() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               now.time_since_epoch())
        .count();
  }
};

inline std::string make_state_payload(double speed, double lean, double lat,
                                      double lon, double heading) {
  std::ostringstream oss;
  oss << "\"speed\":" << speed << ",\"lean\":" << lean << ",\"lat\":" << lat
      << ",\"lon\":" << lon << ",\"heading\":" << heading;
  return oss.str();
}

inline std::string make_event_payload(const std::string &event_type,
                                      const std::string &severity,
                                      double value) {
  std::ostringstream oss;
  oss << "\"event\":\"" << event_type << "\",\"severity\":\"" << severity
      << "\",\"value\":" << value;
  return oss.str();
}

inline std::string make_metrics_payload(double max_speed, double avg_speed,
                                        uint32_t hard_brakes,
                                        uint32_t lean_warnings) {
  std::ostringstream oss;
  oss << "\"max_speed\":" << max_speed << ",\"avg_speed\":" << avg_speed
      << ",\"hard_brakes\":" << hard_brakes
      << ",\"lean_warnings\":" << lean_warnings;
  return oss.str();
}

} // namespace ridersense
