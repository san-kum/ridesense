#pragma once

#include "analytics/event.h"
#include "capture/frame.h"
#include "common/logger.h"
#include "services/service.h"
#include <functional>
#include <unordered_map>

namespace ridersense {

using EventCallback = std::function<void(const Event &)>;

struct EventDetectorConfig {
  double hard_brake_threshold = -6.0;   
  double hard_accel_threshold = 4.0;    
  double max_lean_threshold = 45.0;     
  double close_object_distance = 5.0;   
  double high_slip_threshold = 10.0;    
  double cooldown_seconds = 2.0;        
};

class EventDetector : public Service {
public:
  explicit EventDetector(const EventDetectorConfig &config);
  ~EventDetector() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void process(std::shared_ptr<FusionFrame> fusion,
               std::shared_ptr<DetectionFrame> detections,
               std::shared_ptr<LaneFrame> lanes);

  void set_callback(EventCallback cb) { callback_ = cb; }

private:
  void emit(EventType type, Severity severity, const std::string &desc,
            double value);
  bool can_emit(EventType type);

  EventDetectorConfig config_;
  std::shared_ptr<spdlog::logger> logger_;
  EventCallback callback_;

  std::unordered_map<EventType, Timestamp> last_emit_;
};

} // namespace ridersense
