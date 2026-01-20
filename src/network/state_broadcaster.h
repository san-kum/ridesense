#pragma once

#include "analytics/event.h"
#include "analytics/metrics_collector.h"
#include "capture/frame.h"
#include "common/logger.h"
#include "network/state_message.h"
#include "network/state_server.h"
#include "services/service.h"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace ridersense {

class StateBroadcaster : public Service {
public:
  StateBroadcaster(std::shared_ptr<StateServer> server, double rate_hz);
  ~StateBroadcaster() override;

  bool init() override;
  bool start() override;
  bool stop() override;
  bool shutdown() override;

  void on_fusion(std::shared_ptr<FusionFrame> fusion,
                 std::shared_ptr<GpsFrame> gps);
  void on_event(const Event &event);
  void on_metrics(const RidingMetrics &metrics);

private:
  void broadcast_loop();
  void send_heartbeat();

  std::shared_ptr<StateServer> server_;
  double rate_hz_;
  std::chrono::milliseconds interval_ms_;

  std::shared_ptr<spdlog::logger> logger_;

  std::atomic<bool> running_{false};
  std::thread broadcast_thread_;

  mutable std::mutex state_mutex_;
  double speed_{0.0};
  double lean_{0.0};
  double lat_{0.0};
  double lon_{0.0};
  double heading_{0.0};
  bool state_updated_{false};
};

} // namespace ridersense
